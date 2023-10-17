package frc.robot.commands.autoalign;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.utils.control.DriveToPoseController;

public class DriveToPose extends CommandBase {
    public static final double USE_STOP_MAX_DISTANCE = 1;

    private final Swerve swerve;
    private final Pose2d targetPose;

    private final DriveToPoseController controller;

    public DriveToPose(
            final Swerve swerve,
            final Pose2d targetPose
    ) {
        this.swerve = swerve;
        this.targetPose = targetPose;

        final ProfiledPIDController xController = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
                )
        );
        xController.setTolerance(0.1, 0.1);

        final ProfiledPIDController yController = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
                )
        );
        yController.setTolerance(0.1, 0.1);

        final ProfiledPIDController thetaController = new ProfiledPIDController(
                1, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.TRAJECTORY_MAX_ANGULAR_SPEED,
                        Constants.Swerve.TRAJECTORY_MAX_ANGULAR_ACCELERATION
                )
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(0.1, 0.1);

        this.controller = new DriveToPoseController(xController, yController, thetaController);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        final Pose2d currentPose = swerve.getEstimatedPosition();
        final Gyro gyro = swerve.getGyro();

        final double approxDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        if (approxDistance <= USE_STOP_MAX_DISTANCE) {
            controller.resetWithStop(currentPose, gyro);
        } else {
            controller.reset(currentPose, swerve.getFieldRelativeSpeeds(), gyro);
        }
    }

    @Override
    public void execute() {
        final Pose2d currentPose = swerve.getEstimatedPosition();
        final ChassisSpeeds speeds = controller.calculate(currentPose, targetPose);

        swerve.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
