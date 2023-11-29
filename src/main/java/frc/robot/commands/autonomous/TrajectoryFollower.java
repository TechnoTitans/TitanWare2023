package frc.robot.commands.autonomous;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.auto.DriveController;
import frc.robot.utils.auto.TitanMarker;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.utils.control.DriveToPoseController;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class TrajectoryFollower extends Command {
    //TODO: this max value need to be tuned/verified (will depend on how well our auto pid is)
    public static final double MAX_DISTANCE_DIFF_METERS = 0.3;
    public static final double REJECT_TIME_SECONDS = 1;
    public static boolean HAS_AUTO_RAN = false;
    private final String logKey = "Auto";

    private final TitanTrajectory trajectory;
    private TitanTrajectory transformedTrajectory;
    private final boolean transformForAlliance;
    private final Timer timer;

    private final Swerve swerve;
    private final DriveController holonomicDriveController;
    private final DriveToPoseController holdPositionController;
    private final PhotonVision<?> photonVision;
    private final NavigableMap<Double, TitanMarker> eventMarkerNavigableMap;

    private final SuperstructureStates.CANdleState originalCandleState;
    private final FollowerContext followerContext;

    private boolean isInAuto;

    private EventMarker lastRanMarker;
    private Pose2d holdPosition;

    private boolean hasMarkers;
    private boolean paused;
    private boolean wheelX;

    public TrajectoryFollower(
            final Swerve swerve,
            final DriveController holonomicDriveController,
            final DriveToPoseController holdPositionController,
            final PhotonVision<?> photonVision,
            final TitanTrajectory trajectory,
            final boolean transformForAlliance,
            final FollowerContext followerContext
    ) {
        this.timer = new Timer();
        this.eventMarkerNavigableMap = new TreeMap<>();

        this.swerve = swerve;
        this.holonomicDriveController = holonomicDriveController;
        this.holdPositionController = holdPositionController;
        this.photonVision = photonVision;
        this.trajectory = trajectory;
        this.transformForAlliance = transformForAlliance;

        this.followerContext = followerContext;
        this.originalCandleState = CandleController.getInstance().getCurrentState();
        addRequirements(swerve, followerContext.getClaw(), followerContext.getElevator());
    }

    public void reset() {
        followerContext.setPaused(false);
        followerContext.setWheelX(false);
        paused = false;
        wheelX = false;

        holonomicDriveController.reset();
        holdPositionController.reset(
                photonVision.getEstimatedPosition(), swerve.getFieldRelativeSpeeds(), swerve.getGyro()
        );

        timer.restart();
    }

    @Override
    public void initialize() {
        if (transformForAlliance) {
            transformedTrajectory = TitanTrajectory.transformTrajectoryForAlliance(
                    trajectory, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            );
        } else {
            transformedTrajectory = trajectory;
        }

        final PathPlannerTrajectory.State initialState = transformedTrajectory.getInitialState();
        final Rotation2d initialHolonomicRotation = initialState.targetHolonomicRotation;

        isInAuto = RobotState.isAutonomous();
        if (!TrajectoryFollower.HAS_AUTO_RAN && isInAuto) {
            swerve.setAngle(initialHolonomicRotation);
            photonVision.resetPosition(
                    new Pose2d(initialState.positionMeters, initialHolonomicRotation),
                    initialHolonomicRotation
            );
            TrajectoryFollower.HAS_AUTO_RAN = true;
        }

//        if (Constants.PathPlanner.IS_USING_PATH_PLANNER_SERVER) {
//            PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
//        }

        for (final TitanMarker eventMarker : transformedTrajectory.getEventMarkers()) {
            eventMarkerNavigableMap.put(eventMarker.getTimeSeconds(transformedTrajectory), eventMarker);
        }
        hasMarkers = !eventMarkerNavigableMap.isEmpty();

        Logger.recordOutput(
                logKey + "/Markers",
                eventMarkerNavigableMap.values().stream().map(
                        eventMarker -> new Pose2d(eventMarker.getMarkerPosition(), Rotation2d.fromDegrees(0))
                ).toArray(Pose2d[]::new)
        );

        CandleController.getInstance().setStrobe(SuperstructureStates.CANdleState.BLUE, 0.25);
        reset();
    }

    private void holdPosition(final Pose2d positionToHold, final Pose2d currentPose) {
        final ChassisSpeeds targetChassisSpeeds = holdPositionController.calculate(currentPose, positionToHold);

//        if (Constants.PathPlanner.IS_USING_PATH_PLANNER_SERVER) {
//            PathPlannerServer.sendPathFollowingData(positionToHold, currentPose);
//        }

        Logger.recordOutput(logKey + "/EstimatedPose", new Pose2d(
                currentPose.getTranslation(),
                Rotation2d.fromRadians(MathUtil.angleModulus(currentPose.getRotation().getRadians()))
        ));
        Logger.recordOutput(logKey + "/WantedState", positionToHold);

        swerve.drive(targetChassisSpeeds);
    }

    private void driveToState(final PathPlannerTrajectory.State state, final Pose2d currentPose) {
        final ChassisSpeeds targetChassisSpeeds = holonomicDriveController.calculate(currentPose, state);

//        if (Constants.PathPlanner.IS_USING_PATH_PLANNER_SERVER) {
//            PathPlannerServer.sendPathFollowingData(new Pose2d(
//                    state.poseMeters.getTranslation(),
//                    state.holonomicRotation
//            ), currentPose);
//        }

        Logger.recordOutput(logKey + "/EstimatedPose", new Pose2d(
                currentPose.getTranslation(),
                Rotation2d.fromRadians(MathUtil.angleModulus(currentPose.getRotation().getRadians()))
        ));
        Logger.recordOutput(logKey + "/WantedState", new Pose2d(
                state.positionMeters.getX(),
                state.positionMeters.getY(),
                state.targetHolonomicRotation
        ));

        swerve.drive(targetChassisSpeeds);
    }

    private void wheelX(final boolean wheelX) {
        if (this.wheelX == wheelX) {
            return;
        }

        this.wheelX = wheelX;
        if (wheelX) {
            swerve.wheelX();
        }
    }

    private void dtPause(final boolean paused) {
        if (this.paused == paused) {
            return;
        }

        this.paused = paused;
        if (paused) {
            final Pose2d estimatedPosition = photonVision.getEstimatedPosition();

            this.holdPosition = estimatedPosition;
            this.holdPositionController.resetWithStop(estimatedPosition, swerve.getGyro());
            this.timer.stop();
        } else {
            this.holonomicDriveController.reset();
            this.timer.start();
        }
    }

    @Override
    public void execute() {
        final double currentTime = timer.get();
        final PathPlannerTrajectory.State sample = transformedTrajectory.sample(currentTime);
        final Pose2d currentPose = photonVision.getEstimatedPosition();

        if (hasMarkers) {
             commander(currentPose, currentTime);
        }

        final boolean isWheelX = followerContext.isWheelX();
        final boolean isPaused = followerContext.isPaused();

        Logger.recordOutput(logKey + "/IsWheelX", isWheelX);
        Logger.recordOutput(logKey + "/IsPaused", isPaused);

        wheelX(isWheelX);
        dtPause(isPaused);

        if (isWheelX) {
            swerve.wheelX();
        } else if (isPaused && holdPosition != null) {
            holdPosition(holdPosition, currentPose);
        } else if (!isPaused) {
            driveToState(sample, currentPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        CandleController.getInstance().setState(originalCandleState);
        swerve.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return (isInAuto && !RobotState.isAutonomous())
                || timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
    }

    private void commander(final Pose2d currentPose, final double time) {
        // TODO: does any of this logic here work? test it!
        final Map.Entry<Double, TitanMarker> ceilingEntry =
                eventMarkerNavigableMap.ceilingEntry(time);
        final Map.Entry<Double, TitanMarker> floorEntry =
                eventMarkerNavigableMap.floorEntry(time);

        final TitanMarker nextMarker;
        if (ceilingEntry == null && floorEntry == null) {
            // return early if there aren't any markers left
            return;
        } else if (ceilingEntry == null) {
            nextMarker = floorEntry.getValue();
        } else if (floorEntry == null) {
            nextMarker = ceilingEntry.getValue();
        } else {
            final TitanMarker ceilingMarker = ceilingEntry.getValue();
            final TitanMarker floorMarker = floorEntry.getValue();

            final Translation2d currentTranslation = currentPose.getTranslation();
            final double currentToCeilingDistance = ceilingMarker.getMarkerPosition().getDistance(currentTranslation);
            final double currentToFloorDistance = floorMarker.getMarkerPosition().getDistance(currentTranslation);

            nextMarker = currentToCeilingDistance > currentToFloorDistance ? floorMarker : ceilingMarker;
        }

        final double distanceToNextMarker = nextMarker.getMarkerPosition().getDistance(currentPose.getTranslation());
        final double timeToNextMarker = Math.abs(time - nextMarker.getTimeSeconds(transformedTrajectory));

        if (lastRanMarker == nextMarker
                || distanceToNextMarker > MAX_DISTANCE_DIFF_METERS
                || timeToNextMarker > REJECT_TIME_SECONDS
        ) {
            return;
        } else {
            lastRanMarker = nextMarker;
        }

        final Command command = nextMarker.getCommand();
        command.schedule();
    }

    public static class FollowerContext {
        private final Swerve swerve;
        private final Elevator elevator;
        private final Claw claw;
        private final ChassisSpeeds initialChassisSpeeds;

        private boolean paused;
        private boolean wheelX;

        public FollowerContext(
                final Swerve swerve,
                final Elevator elevator,
                final Claw claw,
                final ChassisSpeeds initialChassisSpeeds
        ) {
            this.swerve = swerve;
            this.elevator = elevator;
            this.claw = claw;
            this.initialChassisSpeeds = initialChassisSpeeds;
        }

        public Swerve getSwerve() {
            return swerve;
        }

        public Elevator getElevator() {
            return elevator;
        }

        public Claw getClaw() {
            return claw;
        }

        public boolean isPaused() {
            return paused;
        }

        public void setPaused(boolean paused) {
            this.paused = paused;
        }

        public boolean isWheelX() {
            return wheelX;
        }

        public void setWheelX(boolean wheelX) {
            this.wheelX = wheelX;
        }

        public ChassisSpeeds getInitialChassisSpeeds() {
            return initialChassisSpeeds;
        }
    }
}