package frc.robot.utils;

@SuppressWarnings("unused")
public class MathMethods {

    // Method for checking if a value is within a specific range using threshold
    public static boolean withinRange(
            final double input,
            final double target,
            final double deadBand
    ) {
        return input > (target - deadBand) && input < (target + deadBand);
    }

    // Method for checking if a value is within a specific range using 2 threshholds
    public static boolean withinUpperAndLower(
            final double input,
            final double target,
            final double lowBound,
            final double upBound
    ) {
        return input > (target - lowBound) && input < (target + upBound);
    }

    // Method for checking if a value is within a specific range using upper and lower limits
    public static boolean withinBand(
            final double input,
            final double lowerLimit,
            final double upperLimit
    ) {
        return input > lowerLimit && input < upperLimit;
    }

    public static boolean withinBand(
            final double val,
            final double band
    ) {
        return Math.abs(val) <= band && Math.abs(val) >= -band;
    }

    public static double deadband(
            final double input,
            final double band
    ) {
        return (Math.abs(input) <= band || Math.abs(input) <= -band) ? 0 : input;
    }

    public static double deadband(
            final double input,
            final double band,
            final double setPoint
    ) {
        return (Math.abs(input) <= band || Math.abs(input) <= -band) ? setPoint : input;
    }
}