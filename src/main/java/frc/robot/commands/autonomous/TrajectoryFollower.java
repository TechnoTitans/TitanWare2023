package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.alignment.GridNode;
import frc.robot.utils.auto.DriveController;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.utils.teleop.ElevatorClawCommand;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.*;

public class TrajectoryFollower extends CommandBase {
    public enum IntakeMode {
        CUBE(SuperstructureStates.ElevatorState.ELEVATOR_CUBE, SuperstructureStates.ClawState.CLAW_ANGLE_CUBE),
        CONE(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY, SuperstructureStates.ClawState.CLAW_INTAKING_CUBE);

        private final SuperstructureStates.ElevatorState elevatorState;
        private final SuperstructureStates.ClawState clawState;

        IntakeMode(final SuperstructureStates.ElevatorState elevatorState, final SuperstructureStates.ClawState clawState) {
            this.elevatorState = elevatorState;
            this.clawState = clawState;
        }

        public SuperstructureStates.ElevatorState getElevatorState() {
            return elevatorState;
        }

        public SuperstructureStates.ClawState getClawState() {
            return clawState;
        }
    }

    //TODO: these 2 max values need to be tuned/verified
    public static final double MAX_TIME_DIFF_SECONDS = 0.1;
    public static final double MAX_DISTANCE_DIFF_METERS = 0.1;
    public static boolean HAS_AUTO_RAN = false;

    private final TitanTrajectory trajectory;
    private TitanTrajectory transformedTrajectory;
    private final boolean transformForAlliance;
    private final Timer timer;

    private final Swerve swerve;
    private final DriveController controller;
    private final PhotonVision photonVision;
    private final NavigableMap<Double, PathPlannerTrajectory.EventMarker> eventMarkerNavigableMap;
    private final Claw claw;
    private final Elevator elevator;
    private final CandleController candleController;

    private boolean isInAuto;

    private PathPlannerTrajectory.EventMarker lastRanMarker;
    private boolean hasMarkers;
    private boolean paused;
    private boolean wheelX;

    public TrajectoryFollower(
            final Swerve swerve,
            final DriveController controller,
            final PhotonVision photonVision,
            final TitanTrajectory trajectory,
            final boolean transformForAlliance,
            final Claw claw,
            final Elevator elevator,
            final CandleController candleController
    ) {
        this.swerve = swerve;
        this.timer = new Timer();
        this.eventMarkerNavigableMap = new TreeMap<>();

        this.controller = controller;
        this.photonVision = photonVision;
        this.trajectory = trajectory;
        this.transformForAlliance = transformForAlliance;

        this.claw = claw;
        this.elevator = elevator;
        this.candleController = candleController;

        addRequirements(swerve, claw, elevator);
    }

    public void reset() {
        paused = false;
        wheelX = false;

        controller.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void initialize() {
        if (transformForAlliance) {
            transformedTrajectory = TitanTrajectory.transformTrajectoryForAlliance(
                    trajectory, DriverStation.getAlliance()
            );
        } else {
            transformedTrajectory = trajectory;
        }

        final PathPlannerTrajectory.PathPlannerState initialState = transformedTrajectory.getInitialState();
        final Rotation2d initialHolonomicRotation = initialState.holonomicRotation;

        isInAuto = RobotState.isAutonomous();

        //TODO: this setAngle call causes loop overruns in sim - seems to be a CTRE implementation issue?
        // investigated and seems like Pigeon2.setYaw() is the culprit, address this eventually
        if (!TrajectoryFollower.HAS_AUTO_RAN && isInAuto) {
            swerve.setAngle(initialHolonomicRotation);
            photonVision.resetPosition(
                    new Pose2d(initialState.poseMeters.getTranslation(), initialHolonomicRotation),
                    initialHolonomicRotation
            );
            TrajectoryFollower.HAS_AUTO_RAN = true;
        }

        if (Constants.PathPlanner.IS_USING_PATH_PLANNER_SERVER) {
            PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
        }

        for (final PathPlannerTrajectory.EventMarker eventMarker : transformedTrajectory.getMarkers()) {
            eventMarkerNavigableMap.put(eventMarker.timeSeconds, eventMarker);
        }
        hasMarkers = !eventMarkerNavigableMap.isEmpty();

        Logger.getInstance().recordOutput(
                "Auto/Markers",
                eventMarkerNavigableMap.values().stream().map(
                        eventMarker -> new Pose2d(eventMarker.positionMeters, Rotation2d.fromDegrees(0))
                ).toArray(Pose2d[]::new)
        );

        reset();

        // TODO: make this better
        candleController.setStrobe(SuperstructureStates.CANdleState.RED, 0.5);
    }

    @Override
    public void execute() {
        final double currentTime = timer.get();
        final PathPlannerTrajectory.PathPlannerState sample =
                (PathPlannerTrajectory.PathPlannerState) transformedTrajectory.sample(currentTime);
        final Pose2d currentPose = photonVision.getEstimatedPosition();

        if (hasMarkers) {
            commander(currentPose, currentTime);
        }

        if (wheelX) {
            swerve.wheelX();
        } else if (!paused) {
            driveToState(sample, currentPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        timer.stop();
        candleController.setState(SuperstructureStates.CANdleState.OFF);
    }

    @Override
    public boolean isFinished() {
        return (isInAuto && !RobotState.isAutonomous())
                || timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
    }

    private void driveToState(final PathPlannerTrajectory.PathPlannerState state, final Pose2d currentPose) {
        final ChassisSpeeds targetChassisSpeeds = controller.calculate(currentPose, state);

        if (Constants.PathPlanner.IS_USING_PATH_PLANNER_SERVER) {
            PathPlannerServer.sendPathFollowingData(new Pose2d(
                    state.poseMeters.getTranslation(),
                    state.holonomicRotation
            ), currentPose);
        }

        Logger.getInstance().recordOutput("Auto/EstimatedPose", new Pose2d(
                currentPose.getTranslation(),
                Rotation2d.fromRadians(MathUtil.angleModulus(currentPose.getRotation().getRadians()))
        ));
        Logger.getInstance().recordOutput("Auto/WantedState", new Pose2d(
                state.poseMeters.getX(),
                state.poseMeters.getY(),
                state.holonomicRotation
        ));

        swerve.drive(targetChassisSpeeds);
    }

    private void wheelX(final boolean wheelX) {
        this.wheelX = wheelX;
        if (wheelX) {
            swerve.wheelX();
        }
    }

    private void dtPause(final boolean paused) {
        this.paused = paused;

        if (paused) {
            timer.stop();
            swerve.stop();
        } else {
            timer.start();
        }
    }

    private void commander(
            final Pose2d currentPose,
            final double time
    ) {
        // TODO: does any of this logic here work? test it!
        //This logic doesnt work, robot seems to be slower than expected so it is getter the farther marker
        final Map.Entry<Double, PathPlannerTrajectory.EventMarker> ceilingEntry =
                eventMarkerNavigableMap.ceilingEntry(time);
        final Map.Entry<Double, PathPlannerTrajectory.EventMarker> floorEntry =
                eventMarkerNavigableMap.floorEntry(time);

        final PathPlannerTrajectory.EventMarker nextMarker;
        if (ceilingEntry == null && floorEntry == null) {
            // return early if there aren't any markers left
            return;
        } else if (ceilingEntry == null) {
            nextMarker = floorEntry.getValue();
        } else if (floorEntry == null) {
            nextMarker = ceilingEntry.getValue();
        } else {
            final PathPlannerTrajectory.EventMarker ceilingMarker = ceilingEntry.getValue();
            final PathPlannerTrajectory.EventMarker floorMarker = floorEntry.getValue();

            final Translation2d currentTranslation = currentPose.getTranslation();
            final double currentToCeilingDistance = ceilingMarker.positionMeters.getDistance(currentTranslation);
            final double currentToFloorDistance = floorMarker.positionMeters.getDistance(currentTranslation);

            nextMarker = currentToCeilingDistance > currentToFloorDistance
                    ? floorMarker
                    : ceilingMarker;
        }

        final double distanceToNextMarker = nextMarker.positionMeters
                .getDistance(currentPose.getTranslation());

        if (lastRanMarker == nextMarker || distanceToNextMarker > MAX_DISTANCE_DIFF_METERS) {
            return;
        } else {
            lastRanMarker = nextMarker;
        }

        final String[] commands = String.join("", nextMarker.names).trim().split(";");
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        for (final String command : commands) {
            final String[] args = command.split(":");
            switch (args[0].toLowerCase()) {
                // TODO: Claw and Elevator calls are to be removed in favor of presets, maybe?
                case "claw" ->
                        commandGroup.addCommands(
                                new ElevatorClawCommand.Builder(elevator, claw)
                                        .withClawState(
                                                SuperstructureStates.ClawState.valueOf(args[1].toUpperCase())
                                        )
                                        .build()
                        );
                case "elevator" ->
                        commandGroup.addCommands(
                                new ElevatorClawCommand.Builder(elevator, claw)
                                        .withElevatorState(
                                                SuperstructureStates.ElevatorState.valueOf(args[1].toUpperCase())
                                        )
                                        .build()
                        );
                case "score" ->
                    commandGroup.addCommands(
                            Commands.runOnce(() -> dtPause(true)),
                            GridNode.buildScoringSequence(
                                    elevator, claw, GridNode.Level.valueOf(args[1].toUpperCase())
                            ),
                            Commands.waitSeconds(0.4),
                            Commands.runOnce(() -> dtPause(false))
                    );
                case "intake" -> {
                    final IntakeMode intakeMode = IntakeMode.valueOf(args[1].toUpperCase());
                    commandGroup.addCommands(
                            new ElevatorClawCommand.Builder(elevator, claw)
                                    .withElevatorClawStates(
                                            intakeMode.getElevatorState(),
                                            intakeMode.getClawState()
                                    )
                                    .build()
                    );
                }
                case "pickup" ->
                        commandGroup.addCommands(
                                new ElevatorClawCommand.Builder(elevator, claw)
                                        .withConditionalClawState(
                                                SuperstructureStates.ClawState.CLAW_INTAKING_CUBE,
                                                SuperstructureStates.ClawState.CLAW_INTAKING_CONE)
                                        .waitUntilState(
                                                SuperstructureStates.ClawState.CLAW_INTAKING_CONE,
                                                0.3
                                        )
                                        .withElevatorClawStates(
                                                SuperstructureStates.ElevatorState.ELEVATOR_STANDBY,
                                                SuperstructureStates.ClawState.CLAW_HOLDING
                                        )
                                        .build()
                        );
                case "autobalance" ->
                        commandGroup.addCommands(new AutoBalance(swerve));
                case "wait" ->
                        commandGroup.addCommands(Commands.waitSeconds(Double.parseDouble(args[1])));
                case "wheelx" ->
                        commandGroup.addCommands(Commands.runOnce(() ->
                                wheelX(Boolean.parseBoolean(args[1])))
                        );
                case "dtpause" ->
                        commandGroup.addCommands(Commands.runOnce(() ->
                                dtPause(Boolean.parseBoolean(args[1])))
                        );
                default -> {}
            }

            commandGroup.schedule();
        }
    }
}