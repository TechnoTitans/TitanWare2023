package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveModuleIO;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
    private final Pigeon2 pigeon;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModuleIO[] swerveModules;
    private final double[] lastSwerveModulePositionRots = {0.0, 0.0, 0.0, 0.0};
    private Pose2d gyroUseOdometryPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static final double USE_SIMULATED_PITCH = 0;
    public static final double USE_SIMULATED_ROLL = 0;


    public GyroIOSim(final Pigeon2 pigeon, final SwerveDriveKinematics kinematics, final SwerveModuleIO[] swerveModules) {
        this.pigeon = pigeon;
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;

        pigeon.getSimState().setSupplyVoltage(12);
        pigeon.getSimState().setPitch(USE_SIMULATED_PITCH);
        pigeon.getSimState().setRoll(USE_SIMULATED_ROLL);
    }

    private void updateGyro() {
        final SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            final double currentSwervePosition = swerveModules[i].getDrivePosition();

            wheelDeltas[i] = new SwerveModulePosition(
                    (currentSwervePosition - lastSwerveModulePositionRots[i]),
                    swerveModules[i].getAngle()
            );
            lastSwerveModulePositionRots[i] = currentSwervePosition;
        }

        final Twist2d wheelDeltasTwist = kinematics.toTwist2d(wheelDeltas);
        gyroUseOdometryPose = gyroUseOdometryPose.exp(wheelDeltasTwist);

        Logger.getInstance().recordOutput("gyroUseOdometryPose", gyroUseOdometryPose);
        Logger.getInstance().recordOutput("wheelDeltasTwistDx", wheelDeltasTwist.dx);
        Logger.getInstance().recordOutput("wheelDeltasTwistDy", wheelDeltasTwist.dy);
        Logger.getInstance().recordOutput("wheelDeltasTwistDTheta", wheelDeltasTwist.dtheta);
        Logger.getInstance().recordOutput("lastSwerveModulePositionRots", lastSwerveModulePositionRots);

        setAngleInternal(gyroUseOdometryPose.getRotation().getDegrees());
    }

    @Override
    public void config() {
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        //TODO: do we need to use MountPose? if so, check that this mount pose is correct
        pigeon2Configuration.MountPose.MountPoseYaw = -90;

        pigeon.getConfigurator().apply(pigeon2Configuration);
    }

    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        updateGyro();

        inputs.hasHardwareFault = pigeon.getFault_Hardware().refresh().getValue();

        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = -getPitch();
        inputs.rollPositionDeg = getRoll();

        inputs.yawVelocityDegPerSec = pigeon.getAngularVelocityZ().refresh().getValue();
        inputs.pitchVelocityDegPerSec = -pigeon.getAngularVelocityX().refresh().getValue();
        inputs.rollVelocityDegPerSec = pigeon.getAngularVelocityY().refresh().getValue();
    }

    @Override
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    @Override
    public boolean isReal() { return false; }

    @Override
    public double getYaw() {
        updateGyro();
        return pigeon.getYaw().refresh().getValue();
    }

    @Override
    public double getYawBlocking() {
        updateGyro();
        return pigeon.getYaw().waitForUpdate(Constants.LOOP_PERIOD_SECONDS).getValue();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch().refresh().getValue();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll().refresh().getValue();
    }

    @Override
    public Rotation2d getYawRotation2d() {
       return Rotation2d.fromDegrees(getYaw());
    }

    @Override
    public Rotation2d getRotation2dBlocking() {
        return Rotation2d.fromDegrees(getYawBlocking());
    }

    private void setAngleInternal(final double angle) {
        pigeon.getSimState().setRawYaw(angle);
    }

    @Override
    public void setAngle(final double angle) {
        pigeon.setYaw(angle);
    }
}
