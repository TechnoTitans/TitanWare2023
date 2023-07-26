package frc.robot.utils.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;

public class Phoenix5Utils {
    public static int rotationsToCTREPhoenix5NativeUnits(final double rotations) {
        return (int)(rotations * Constants.CTRE.PHOENIX_5_CANCODER_TICKS_PER_ROTATION);
    }

    /**
     * Converts a Phoenix 6 control input (ex. rotations) into Phoenix 5 native units taking a Phoenix 5
     * {@link ControlMode} into account (i.e. PercentOutput -> DutyCycle are 1:1)
     * @param controlMode the Phoenix 5 {@link ControlMode}
     * @param input the Phoenix 6 control input
     * @return the Phoenix 5 control input
     * @see ControlMode
     */
    public static double getPhoenix6To5ControlInput(final ControlMode controlMode, final double input) {
        return switch (controlMode) {
            case PercentOutput -> input;
            case Position, Velocity -> rotationsToCTREPhoenix5NativeUnits(input);
            default -> throw new UnsupportedOperationException("not implemented yet!");
        };
    }
}
