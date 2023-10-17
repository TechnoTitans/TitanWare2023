package frc.robot.commands.autoalign;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.gyro.Gyro;

public class LineUpToGrid extends CommandBase {
    public static final double PAST_CHARGE_STATION_X = 2.3;
    public static final double ALLOW_GRID_LATERAL_MOVEMENT_X = 2.3;

    private final Swerve swerve;
    private final Pose2d targetPose;

    private final ProfiledPIDController xController, yController;
    private final ProfiledPIDController thetaController;

    private boolean xShouldControl = false;
    private boolean yShouldControl = false;

    public LineUpToGrid(
            final Swerve swerve,
            final Pose2d targetPose
    ) {
        this.swerve = swerve;
        this.targetPose = targetPose;

        this.xController = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
                )
        );
        this.xController.setTolerance(0.1, 0.1);

        this.yController = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
                )
        );
        this.yController.setTolerance(0.1, 0.1);

        this.thetaController = new ProfiledPIDController(
                1, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.TRAJECTORY_MAX_ANGULAR_SPEED,
                        Constants.Swerve.TRAJECTORY_MAX_ANGULAR_ACCELERATION
                )
        );
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.thetaController.setTolerance(0.1, 0.1);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        final Pose2d currentPose = swerve.getEstimatedPosition();
        final ChassisSpeeds swerveChassisSpeeds = swerve.getFieldRelativeSpeeds();

        this.xController.reset(
                currentPose.getX(), swerveChassisSpeeds.vxMetersPerSecond
        );

        this.yController.reset(
                currentPose.getY(), swerveChassisSpeeds.vyMetersPerSecond
        );

        final Gyro gyro = swerve.getGyro();
        this.thetaController.reset(
                gyro.getYawRotation2d().getRadians(), gyro.getYawVelocityRotation2d().getRadians()
        );
    }

    @Override
    public void execute() {
        final Pose2d currentPose = swerve.getEstimatedPosition();
        final ChassisSpeeds fieldRelativeSpeeds = swerve.getFieldRelativeSpeeds();

        final double xCurrent = currentPose.getX();
        final double yCurrent = currentPose.getY();

        // x controller
        final boolean newXShouldControl = yController.atGoal() || xCurrent >= PAST_CHARGE_STATION_X;
        if (xShouldControl != newXShouldControl) {
            xController.reset(xCurrent, fieldRelativeSpeeds.vxMetersPerSecond);
            xShouldControl = newXShouldControl;
        }

        final double xControllerInput = xController.calculate(
                xCurrent,
                targetPose.getX()
        );
        final double xInput = xShouldControl ? xControllerInput : 0;

        // y controller
        final boolean newYShouldControl = xCurrent < ALLOW_GRID_LATERAL_MOVEMENT_X;
        if (yShouldControl != newYShouldControl) {
            yController.reset(yCurrent, fieldRelativeSpeeds.vyMetersPerSecond);
            yShouldControl = newYShouldControl;
        }

        final double yControllerInput = yController.calculate(
                yCurrent,
                targetPose.getY()
        );
        final double yInput = yShouldControl ? yControllerInput : 0;

        final double rotInput = thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()
        );

        swerve.drive(
                xInput,
                yInput,
                rotInput,
                true
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
