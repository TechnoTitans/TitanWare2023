package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(final Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        inputs.hasHardwareFault = pigeon.getFault_Hardware().refresh().getValue();

        inputs.rollPositionDeg = getRoll();
        inputs.pitchPositionDeg = -getPitch();
        inputs.yawPositionDeg = getHeading();

        inputs.rollVelocityDegPerSec = pigeon.getAngularVelocityY().refresh().getValue();
        inputs.pitchVelocityDegPerSec = -pigeon.getAngularVelocityX().refresh().getValue();
        inputs.yawVelocityDegPerSec = pigeon.getAngularVelocityZ().refresh().getValue();
    }

    @Override
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    @Override
    public boolean isReal() { return true; }

    @Override
    public double getHeading() {
        return pigeon.getYaw().refresh().getValue();
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
    public void setAngle(final double angle) {
        pigeon.setYaw(angle);
    }
}
