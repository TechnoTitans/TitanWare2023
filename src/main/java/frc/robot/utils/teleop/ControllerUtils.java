package frc.robot.utils.teleop;

import edu.wpi.first.math.MathUtil;

public class ControllerUtils {
    public static double getStickInput(
            final double input,
            final double deadband,
            final double scalar,
            final double sensitivity
    ) {
        // yes, this negative sign does need to exist because controller stick up is -1 not 1
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
