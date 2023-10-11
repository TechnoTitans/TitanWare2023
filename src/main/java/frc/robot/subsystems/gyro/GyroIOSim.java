package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.utils.ctre.Phoenix6Utils;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
    private final Pigeon2 pigeon;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] swerveModules;
    private final double[] lastSwerveModulePositionMeters = {0.0, 0.0, 0.0, 0.0};

    private Pose2d gyroUseOdometryPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static final double USE_SIMULATED_PITCH = 0;
    public static final double USE_SIMULATED_ROLL = 0;

    public GyroIOSim(final Pigeon2 pigeon, final SwerveDriveKinematics kinematics, final SwerveModule[] swerveModules) {
        this.pigeon = pigeon;
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;

        config();

        pigeon.getSimState().setSupplyVoltage(12);
        pigeon.getSimState().setPitch(USE_SIMULATED_PITCH);
        pigeon.getSimState().setRoll(USE_SIMULATED_ROLL);
    }

    private void updateGyro() {
        final SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            final SwerveModulePosition currentSwervePosition = swerveModules[i].getPosition();

            wheelDeltas[i] = new SwerveModulePosition(
                    (currentSwervePosition.distanceMeters - lastSwerveModulePositionMeters[i]),
                    currentSwervePosition.angle
            );
            lastSwerveModulePositionMeters[i] = currentSwervePosition.distanceMeters;
        }

        final Twist2d wheelDeltasTwist = kinematics.toTwist2d(wheelDeltas);
        gyroUseOdometryPose = gyroUseOdometryPose.exp(wheelDeltasTwist);

        Logger.getInstance().recordOutput(Gyro.logKey + "/GyroUseOdometryPose", gyroUseOdometryPose);
        Logger.getInstance().recordOutput(Gyro.logKey + "/WheelDeltasTwistDx", wheelDeltasTwist.dx);
        Logger.getInstance().recordOutput(Gyro.logKey + "/WheelDeltasTwistDy", wheelDeltasTwist.dy);
        Logger.getInstance().recordOutput(Gyro.logKey + "/WheelDeltasTwistDTheta", wheelDeltasTwist.dtheta);
        Logger.getInstance().recordOutput(Gyro.logKey + "/LastSwerveModulePositionRots", lastSwerveModulePositionMeters);

        setAngleInternal(gyroUseOdometryPose.getRotation().getDegrees());
    }

    @Override
    public void config() {
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        //TODO: do we need to use MountPose?
        pigeon2Configuration.MountPose.MountPoseYaw = 0;

        pigeon.getConfigurator().apply(pigeon2Configuration);
    }

    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        updateGyro();

        inputs.hasHardwareFault = pigeon.getFault_Hardware().refresh().getValue();

        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = getPitch();
        inputs.rollPositionDeg = getRoll();

        inputs.yawVelocityDegPerSec = getYawVelocitySignal().refresh().getValue();
        inputs.pitchVelocityDegPerSec = getPitchVelocitySignal().refresh().getValue();
        inputs.rollVelocityDegPerSec = getRollVelocitySignal().refresh().getValue();
    }

    public double getYaw() {
        updateGyro();
//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                return pigeon.getYaw().refresh().getValue();
//                getYawVelocitySignal()
//        );
    }

    public StatusSignal<Double> getYawVelocitySignal() {
        return pigeon.getAngularVelocityZ();
    }

    public double getPitch() {
        updateGyro();
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                pigeon.getPitch(),
                getPitchVelocitySignal()
        );
    }

    public StatusSignal<Double> getPitchVelocitySignal() {
        return pigeon.getAngularVelocityX();
    }

    public double getRoll() {
        updateGyro();
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                pigeon.getRoll(),
                getRollVelocitySignal()
        );
    }

    public StatusSignal<Double> getRollVelocitySignal() {
        return pigeon.getAngularVelocityY();
    }

    private void setAngleInternal(final double angle) {
        pigeon.getSimState().setRawYaw(angle);
    }

    @Override
    public void setAngle(final Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}
