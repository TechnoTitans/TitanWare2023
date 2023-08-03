package frc.robot.utils.control;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.gyro.Gyro;

public record DriveToPoseController(
        ProfiledPIDController xController,
        ProfiledPIDController yController,
        ProfiledPIDController thetaController
) {
    public record DriveToPoseOutput(double dx, double dy, double dTheta) {

    }

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

    public DriveToPoseOutput getOutput(
            final double xMeasurement, final double xSetpoint,
            final double yMeasurement, final double ySetpoint,
            final double thetaMeasurement, final double thetaSetpoint
    ) {
        return new DriveToPoseOutput(
                getX(xMeasurement, xSetpoint),
                getY(yMeasurement, ySetpoint),
                getTheta(thetaMeasurement, thetaSetpoint)
        );
    }

    public DriveToPoseOutput getOutput(final Pose2d currentPose, final Pose2d targetPose) {
        return getOutput(
                currentPose.getX(), targetPose.getX(),
                currentPose.getY(), targetPose.getY(),
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()
        );
    }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
}
