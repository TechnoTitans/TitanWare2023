package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class TrajectoryFollower extends CommandBase {
    //TODO: these 2 max values need to be tuned/verified
    public static final double MAX_TIME_DIFF_SECONDS = 0.1;
    public static final double MAX_DISTANCE_DIFF_METERS = 0.05;
    public static final boolean USE_ODOMETRY_FOR_MARKERS = true;
    public static boolean HAS_AUTO_RAN = false;

    private final TitanTrajectory trajectory;
    private TitanTrajectory transformedTrajectory;
    private final boolean transformForAlliance;
    private final Timer timer;

    private final Swerve swerve;
    private final DriveController controller;
    private final PhotonVision photonVision;
    private final List<PathPlannerTrajectory.EventMarker> eventMarkers;
    private final Claw claw;
    private final Elevator elevator;
    private final CandleController candleController;

    private boolean isInAuto;

    private boolean hasMarkers;
    private int markerIndex;
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
        this.eventMarkers = new ArrayList<>(trajectory.getMarkers().size());

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
        markerIndex = 0;
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

        eventMarkers.addAll(transformedTrajectory.getMarkers());
        hasMarkers = !eventMarkers.isEmpty();

        Logger.getInstance().recordOutput(
                "Auto/Markers",
                eventMarkers.stream().map(
                        eventMarker -> new Pose2d(eventMarker.positionMeters, Rotation2d.fromDegrees(0))
                ).toArray(Pose2d[]::new)
        );

        reset();

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

    private void dtPause(final boolean paused) {
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
        // if we're already completed the last marker in the trajectory, then just return early
        if (markerIndex >= eventMarkers.size()) {
            return;
        }

        final Optional<PathPlannerTrajectory.EventMarker> useMarker;
        final PathPlannerTrajectory.EventMarker currentMarker = eventMarkers.get(markerIndex);

        final double currentTimeDiff = Math.abs(currentMarker.timeSeconds - time);
        final double distanceToCurrentMarker = currentMarker.positionMeters
                .getDistance(currentPose.getTranslation());

        // check if we still have another marker to look at
        if (eventMarkers.size() > (markerIndex + 1)) {
            final PathPlannerTrajectory.EventMarker nextMarker = eventMarkers.get(markerIndex + 1);

            if (USE_ODOMETRY_FOR_MARKERS) {
                final double distanceToNextMarker = nextMarker.positionMeters
                        .getDistance(currentPose.getTranslation());

                // if we're closer in distance to the next marker and the next marker is within MAX_DISTANCE_DIFF_METERS,
                // then run the next marker and increment markerIndex
                if ((distanceToNextMarker < distanceToCurrentMarker)
                                && (distanceToNextMarker < MAX_DISTANCE_DIFF_METERS)
                ) {
                    useMarker = Optional.of(nextMarker);
                    markerIndex++;
                } else if (distanceToCurrentMarker < MAX_DISTANCE_DIFF_METERS) {
                    // if we're still within MAX_DISTANCE_DIFF_METERS to the current marker,
                    // then run the current marker
                    useMarker = Optional.of(currentMarker);
                    markerIndex++;
                } else {
                    useMarker = Optional.empty();
                }
            } else {
                // if we're closer in time to the next marker and the next marker is within MAX_TIME_DIFF_SECONDS,
                // then run the next marker and increment markerIndex
                final double nextTimeDiff = Math.abs(nextMarker.timeSeconds - time);

                if ((nextTimeDiff < currentTimeDiff) && (nextTimeDiff < MAX_TIME_DIFF_SECONDS)) {
                    useMarker = Optional.of(nextMarker);
                    markerIndex++;
                } else if (currentTimeDiff < MAX_TIME_DIFF_SECONDS) {
                    // if we're still within MAX_TIME_DIFF_SECONDS to the current marker,
                    // then run the current marker
                    useMarker = Optional.of(currentMarker);
                    markerIndex++;
                } else {
                    useMarker = Optional.empty();
                }
            }
        } else if (
                USE_ODOMETRY_FOR_MARKERS
                        ? (distanceToCurrentMarker < MAX_DISTANCE_DIFF_METERS)
                        : (currentTimeDiff < MAX_TIME_DIFF_SECONDS)
        ) {
            // If we're using odometry for markers, then check if the distance allows us to still use the current marker
            // if not, then check if the time allows us to still use the current marker
            useMarker = Optional.of(currentMarker);
            markerIndex++;
        } else {
            useMarker = Optional.empty();
        }

        if (useMarker.isPresent()) {
            final String[] commands = useMarker.get().names.get(0).trim().split(";");
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
                                Commands.waitSeconds(0.3),
                                Commands.runOnce(() -> dtPause(false))
                        );
                    case "autobalance" ->
                            commandGroup.addCommands(new AutoBalance(swerve));
                    case "wait" ->
                            commandGroup.addCommands(Commands.waitSeconds(Double.parseDouble(args[1])));
                    case "wheelx" ->
                            commandGroup.addCommands(Commands.runOnce(() -> wheelX = Boolean.parseBoolean(args[1])));
                    case "dtpause" ->
                            commandGroup.addCommands(Commands.runOnce(() ->
                                    dtPause(paused = Boolean.parseBoolean(args[1])))
                            );
                    default -> {}
                }
            }

            commandGroup.schedule();
        }
    }
}