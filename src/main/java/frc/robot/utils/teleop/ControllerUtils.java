package frc.robot.utils.teleop;

import edu.wpi.first.math.MathUtil;

public class ControllerUtils {
    public static double getStickInput(
            final double input,
            final double deadband,
            final double scalar,
            final double sensitivity
    ) {
        return -MathUtil.applyDeadband(input, Math.abs(deadband)) * scalar * sensitivity;
    }

    public static double getStickInputWithWeight(
            final double input,
            final double deadband,
            final double scalar,
            final double sensitivity,
            final double weight
    ) {
        return getStickInput(input, deadband, scalar, sensitivity) * weight;
    }
}
