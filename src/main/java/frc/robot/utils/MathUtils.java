package frc.robot.utils;

public class MathUtils {
    /**
     * Check whether a measured value is within tolerance of a setpoint
     * @param measured the measured value
     * @param setpoint the setpoint
     * @param tolerance the tolerance
     * @return true if within tolerance, false if not
     */
    public static boolean withinTolerance(
            final double measured,
            final double setpoint,
            final double tolerance
    ) {
        return Math.abs(measured - setpoint) < tolerance;
    }
}