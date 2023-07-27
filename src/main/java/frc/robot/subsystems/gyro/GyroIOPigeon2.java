package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.utils.ctre.Phoenix6Utils;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(final Pigeon2 pigeon) {
        this.pigeon = pigeon;
        config();
    }

    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        inputs.hasHardwareFault = pigeon.getFault_Hardware().refresh().getValue();

        inputs.rollPositionDeg = getRoll();
        inputs.pitchPositionDeg = getPitch();
        inputs.yawPositionDeg = getYaw();

        inputs.rollVelocityDegPerSec = getRollVelocitySignal().refresh().getValue();
        inputs.pitchVelocityDegPerSec = getPitchVelocitySignal().refresh().getValue();
        inputs.yawVelocityDegPerSec = getYawVelocitySignal().refresh().getValue();
    }

    @Override
    public void config() {
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        //TODO: do we need to use MountPose? if so, check that this mount pose is correct
        pigeon2Configuration.MountPose.MountPoseYaw = 0;

        pigeon.getConfigurator().apply(pigeon2Configuration);
    }

    public double getYaw() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                pigeon.getYaw(),
                getYawVelocitySignal()
        );
    }

    public StatusSignal<Double> getYawVelocitySignal() {
        return pigeon.getAngularVelocityZ();
    }

    public double getPitch() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                pigeon.getPitch(),
                getPitchVelocitySignal()
        );
    }

    public StatusSignal<Double> getPitchVelocitySignal() {
        return pigeon.getAngularVelocityX();
    }

    public double getRoll() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                pigeon.getRoll(),
                getRollVelocitySignal()
        );
    }

    public StatusSignal<Double> getRollVelocitySignal() {
        return pigeon.getAngularVelocityY();
    }

    @Override
    public void setAngle(final double angle) {
        pigeon.setYaw(angle);
    }
}
