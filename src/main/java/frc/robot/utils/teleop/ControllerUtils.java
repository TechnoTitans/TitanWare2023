package frc.robot.utils.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ControllerUtils {
    public final static double STICK_TO_GYRO_ROTATION = -1;
    public final static Rotation2d STICK_TO_FIELD_RELATIVE_ROTATION = Rotation2d.fromDegrees(-90);

    public static double getStickInput(
            final double input,
            final double deadband
    ) {
        // yes, this negative sign does need to exist because controller stick up is -1 not 1
        return -MathUtil.applyDeadband(input, Math.abs(deadband));
    }

    public static double getStickInput(
            final double input,
            final double deadband,
            final double scalar,
            final double sensitivity
    ) {
        return getStickInput(input, deadband) * scalar * sensitivity;
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

    public static Rotation2d getFieldRelativeAngleFromStickInputs(final double xInput, final double yInput) {
        return Rotation2d.fromRadians(Math.atan2(yInput, xInput))
                .rotateBy(STICK_TO_FIELD_RELATIVE_ROTATION)
                .times(STICK_TO_GYRO_ROTATION);
    }
}
