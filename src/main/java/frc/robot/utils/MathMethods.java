package frc.robot.utils;

@SuppressWarnings("unused")
public class MathMethods {

    // Method for checking if a value is within a specific range using threshold
    public static boolean withinRange(double input, double target, double deadBand) {
        return input > (target-deadBand) && input < (target+deadBand);
    }

    // Method for checking if a value is within a specific range using 2 threshholds
    public static boolean withinUpperAndLower(double input, double target, double lowBound, double upBound) {
        return input > (target-lowBound) && input < (target+upBound);
    }

    // Method for checking if a value is within a specific range using upper and lower limits
    public static boolean withinBand(double input, double lowerLimit, double upperLimit) {
        return input > lowerLimit && input < upperLimit;
    }

}
