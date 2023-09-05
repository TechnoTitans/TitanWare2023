package frc.robot.utils.control;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.gyro.Gyro;

public record DriveToPoseController(
        ProfiledPIDController xController,
        ProfiledPIDController yController,
        ProfiledPIDController thetaController
) {
    public DriveToPoseController(
            final ProfiledPIDController xController,
            final ProfiledPIDController yController,
            final ProfiledPIDController thetaController
    ) {
        this.xController = xController;
        this.yController = yController;

        this.thetaController = thetaController;
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void resetX(final double xMeasurement, final double dxMeasurement) {
        this.xController.reset(xMeasurement, dxMeasurement);
    }

    public void resetY(final double yMeasurement, final double dyMeasurement) {
        this.yController.reset(yMeasurement, dyMeasurement);
    }

    public void resetTheta(final double thetaMeasurement, final double dThetaMeasurement) {
        this.thetaController.reset(thetaMeasurement, dThetaMeasurement);
    }

    public void reset(final Pose2d currentPose, final ChassisSpeeds fieldRelativeSpeeds, final Gyro gyro) {
        resetX(currentPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
        resetY(currentPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
        resetTheta(gyro.getYawRotation2d().getRadians(), gyro.getYawVelocityRotation2d().getRadians());
    }

    public void resetWithStop(final Pose2d currentPose, final Gyro gyro) {
        resetX(currentPose.getX(), 0);
        resetY(currentPose.getY(), 0);
        resetTheta(gyro.getYawRotation2d().getRadians(), 0);
    }

    public double getX(final double xMeasurement, final double xSetpoint) {
        return this.xController.calculate(xMeasurement, xSetpoint);
    }

    public double getY(final double yMeasurement, final double ySetpoint) {
        return this.yController.calculate(yMeasurement, ySetpoint);
    }

    public double getTheta(final double thetaMeasurement, final double thetaSetpoint) {
        return this.thetaController.calculate(thetaMeasurement, thetaSetpoint);
    }

    public ChassisSpeeds calculate(final Pose2d currentPose, final Pose2d targetPose, final boolean fieldRelative) {
        final Rotation2d currentRotation = currentPose.getRotation();
        final Rotation2d targetRotation = targetPose.getRotation();

        final double vx = getX(currentPose.getX(), targetPose.getX());
        final double vy = getY(currentPose.getY(), targetPose.getY());
        final double omegaTheta = getTheta(currentRotation.getRadians(), targetRotation.getRadians());

        return fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaTheta, currentRotation)
                : new ChassisSpeeds(vx, vy, omegaTheta);
    }

    public ChassisSpeeds calculate(final Pose2d currentPose, final Pose2d targetPose) {
        return calculate(currentPose, targetPose, true);
    }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
}
