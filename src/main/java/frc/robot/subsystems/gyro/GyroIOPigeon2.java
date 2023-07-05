package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(final Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        inputs.hasHardwareFault = pigeon.getFault_Hardware().refresh().getValue();

        inputs.rollPositionRad = Units.degreesToRadians(getRoll());
        inputs.pitchPositionRad = Units.degreesToRadians(-getPitch());
        inputs.yawPositionRad = Units.degreesToRadians(getHeading());

        inputs.rollVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityY().refresh().getValue());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-pigeon.getAngularVelocityX().refresh().getValue());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZ().refresh().getValue());
    }

    @Override
    public Pigeon2 getPigeon() {
        return pigeon;
    }

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
