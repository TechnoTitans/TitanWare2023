package frc.robot.utils.teleop;

import edu.wpi.first.math.MathUtil;

public class ControllerUtils {
    public static double getStickInput(
            final double input,
            final double deadband,
            final double scalar,
            final double sensitivity
    ) {
        //TODO: figure out if we need to negate this value here...
        // it seems like when we push the sticks on an xbox controller UP, the actual reflected value of the stick axis
        // is -1

        //TODO: update -> negating the stick input seems correct and we *should* continue doing it to make UP positive
        // (1) instead of negative (-1), the direction issue likely lies within motor inverts - check those
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
