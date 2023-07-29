package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.auto.AutoOption;
import frc.robot.utils.auto.DriveController;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class TrajectoryManager {
    private final Swerve swerve;
    private final DriveController controller;
    private final PhotonVision photonVision;

    private final Claw claw;
    private final Elevator elevator;

    public TrajectoryManager(
            final Swerve swerve,
            final DriveController controller,
            final PhotonVision photonVision,
            final Claw claw,
            final Elevator elevator
    ) {
        this.swerve = swerve;
        this.controller = controller;
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
        return TitanTrajectory.fromPathPlannerTrajectory(
                PathPlanner.loadPath(trajectoryDir, maxVel, maxAccel, reverseTrajectory)
        );
    }

    public TitanTrajectory getTrajectoryFromPath(final AutoOption autoOption) {
        return getTrajectoryFromPath(
                autoOption.pathName(), autoOption.maxVelocity(), autoOption.maxAcceleration(), false
        );
    }

    public TrajectoryFollower getCommand(final AutoOption autoOption) {
        final TitanTrajectory trajectory = getTrajectoryFromPath(autoOption);
        return new TrajectoryFollower(
                swerve, controller, photonVision, trajectory, true, claw, elevator
        );
    }

    public TrajectoryFollower getCommand(final PathPlannerTrajectory pathPlannerTrajectory) {
        return new TrajectoryFollower(
                swerve,
                controller,
                photonVision,
                TitanTrajectory.fromPathPlannerTrajectory(pathPlannerTrajectory),
                false,
                claw,
                elevator
        );
    }

    public void follow(final AutoOption autoOption) {
        getCommand(autoOption).schedule();
    }
}