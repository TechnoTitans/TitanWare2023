package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.auto.AutoOption;
import frc.robot.utils.auto.DriveController;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.utils.control.DriveToPoseController;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class TrajectoryManager {
    private final Swerve swerve;
    private final DriveController controller;
    private final DriveToPoseController holdPositionController;
    private final CandleController candleController;
    private final PhotonVision photonVision;

    private final Claw claw;
    private final Elevator elevator;

    public TrajectoryManager(
            final Swerve swerve,
            final DriveController controller,
            final DriveToPoseController holdPositionController,
            final CandleController candleController,
            final PhotonVision photonVision,
            final Claw claw,
            final Elevator elevator
    ) {
        this.swerve = swerve;
        this.controller = controller;
        this.holdPositionController = holdPositionController;
        this.candleController = candleController;
        this.photonVision = photonVision;

        this.claw = claw;
        this.elevator = elevator;
    }

    public TitanTrajectory getTrajectoryFromPath(
            final String trajectoryDir,
            final double maxVel,
            final double maxAccel,
            final boolean reverseTrajectory
    ) {
        final TrajectoryFollower.FollowerContext followerContext =
                new TrajectoryFollower.FollowerContext(elevator, claw);

        return TitanTrajectory.fromPathPlannerTrajectory(
                PathPlanner.loadPath(trajectoryDir, maxVel, maxAccel, reverseTrajectory),
                followerContext
        );
    }

    public TitanTrajectory getTrajectoryFromPath(final AutoOption autoOption) {
        return getTrajectoryFromPath(
                autoOption.pathName(), autoOption.maxVelocity(), autoOption.maxAcceleration(), false
        );
    }

    public TrajectoryFollower getCommand(final TitanTrajectory trajectory) {
        // TODO: what happens if transformForAlliance is true?
        return new TrajectoryFollower(
                swerve,
                candleController,
                controller,
                holdPositionController,
                photonVision,
                trajectory,
                false,
                trajectory.getFollowerContext()
        );
    }

    public TrajectoryFollower getCommand(final AutoOption autoOption) {
        final TitanTrajectory trajectory = getTrajectoryFromPath(autoOption);
        return new TrajectoryFollower(
                swerve,
                candleController,
                controller,
                holdPositionController,
                photonVision,
                trajectory,
                true,
                trajectory.getFollowerContext()
        );
    }

    public void follow(final AutoOption autoOption) {
        getCommand(autoOption).schedule();
    }
}