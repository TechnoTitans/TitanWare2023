package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.DriveController;
import frc.robot.utils.auto.AutoOption;
import frc.robot.utils.auto.TitanTrajectory;

public class TrajectoryManager {
    private final Swerve swerve;
    private final DriveController controller;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final Claw claw;
    private final Elevator elevator;

    public TrajectoryManager(Swerve swerve, DriveController controller, SwerveDrivePoseEstimator poseEstimator, Claw claw, Elevator elevator) {
        this.swerve = swerve;
        this.controller = controller;
        this.poseEstimator = poseEstimator;

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
                swerve, controller, poseEstimator, trajectory, true, claw, elevator
        );
    }

    public void follow(final AutoOption autoOption) {
        getCommand(autoOption).schedule();
    }
}