package frc.robot.utils.sim.feedback;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import frc.robot.utils.ctre.Phoenix5Utils;

public class SimPhoenix5CANCoder implements SimFeedbackSensor {
    private final CANCoder canCoder;
    private final CANCoderSimCollection simCollection;

    public SimPhoenix5CANCoder(final CANCoder canCoder) {
        this.canCoder = canCoder;
        this.simCollection = canCoder.getSimCollection();
    }

    @Override
    public void setSupplyVoltage(double volts) {
        Phoenix5Utils.reportIfNotOk(canCoder.getDeviceID(), simCollection.setBusVoltage(volts));
    }

    @Override
    public void setRawPosition(double rotations) {
        Phoenix5Utils.reportIfNotOk(
                canCoder.getDeviceID(),
                simCollection.setRawPosition(Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(rotations))
        );
    }

    @Override
    public void addPosition(double deltaRotations) {
        Phoenix5Utils.reportIfNotOk(
                canCoder.getDeviceID(),
                simCollection.addPosition(Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(deltaRotations))
        );
    }

    @Override
    public void setVelocity(double rotationsPerSec) {
        Phoenix5Utils.reportIfNotOk(
                canCoder.getDeviceID(),
                simCollection.setVelocity(
                        Phoenix5Utils.rotationsPerSecToCTREPhoenix5NativeUnitsPer100Ms(rotationsPerSec)
                )
        );
    }
}
