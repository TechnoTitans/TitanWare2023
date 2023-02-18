package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class TestTraj {
    private final Swerve swerve;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public TestTraj(Swerve swerve, SwerveDriveKinematics kinematics, SwerveDriveOdometry swerveDriveOdometry) {
        this.swerve = swerve;
        this.kinematics = kinematics;
        this.odometry = swerveDriveOdometry;
    }

    public Command followPPTrajectory(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath) {
                        odometry.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        odometry::getPoseMeters,
                        kinematics,
                        new PIDController(2.5, 0, 0),
                        new PIDController(2.5, 0, 0),
                        new PIDController(0, 0, 0),
                        swerve.getModuleStateConsumer(),
                        true),
                new InstantCommand(swerve::stop)
        );
    }
}
