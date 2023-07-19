package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(final Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        inputs.hasHardwareFault = pigeon.getFault_Hardware().refresh().getValue();

        inputs.rollPositionDeg = getRoll();
        inputs.pitchPositionDeg = -getPitch();
        inputs.yawPositionDeg = getYaw();

        inputs.rollVelocityDegPerSec = pigeon.getAngularVelocityY().refresh().getValue();
        inputs.pitchVelocityDegPerSec = -pigeon.getAngularVelocityX().refresh().getValue();
        inputs.yawVelocityDegPerSec = pigeon.getAngularVelocityZ().refresh().getValue();
    }

    @Override
    public void config() {
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        //TODO: do we need to use MountPose? if so, check that this mount pose is correct
        pigeon2Configuration.MountPose.MountPoseYaw = -90;

        pigeon.getConfigurator().apply(pigeon2Configuration);
    }

    @Override
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    @Override
    public boolean isReal() { return true; }

    @Override
    public double getYaw() {
        return pigeon.getYaw().refresh().getValue();
    }

    @Override
    public double getYawBlocking() {
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

    @Override
    public void setAngle(final double angle) {
        pigeon.setYaw(angle);
    }
}
