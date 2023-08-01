package frc.robot.utils.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

/**
 * Simulation shared utility methods/functions
 */
public class SimUtils {

    public static void setCTRETalonFXSimStateMotorInverted(
            final CoreTalonFX talonFX,
            final InvertedValue invertedValue
    ) {
        switch (invertedValue) {
            case Clockwise_Positive ->
                    talonFX.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            case CounterClockwise_Positive ->
                    talonFX.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        }
    }

    public static void setCTRECANCoderSimStateSensorDirection(
            final CANcoder canCoder,
            final SensorDirectionValue sensorDirectionValue
    ) {
        switch (sensorDirectionValue) {
            case Clockwise_Positive ->
                    canCoder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            case CounterClockwise_Positive ->
                    canCoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        }
    }
}
