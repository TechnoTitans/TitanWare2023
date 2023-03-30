package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

@SuppressWarnings("unused")
public class MathMethods {

    // Method for checking if a value is within a specific range using threshold
    public static boolean withinRange(double input, double target, double deadBand) {
        return input > (target - deadBand) && input < (target + deadBand);
    }

    // Method for checking if a value is within a specific range using 2 threshholds
    public static boolean withinUpperAndLower(double input, double target, double lowBound, double upBound) {
        return input > (target - lowBound) && input < (target + upBound);
    }

    // Method for checking if a value is within a specific range using upper and lower limits
    public static boolean withinBand(double input, double lowerLimit, double upperLimit) {
        return input > lowerLimit && input < upperLimit;
    }

    public static boolean withinBand(double val, double band) {
        return Math.abs(val) <= band && Math.abs(val) >= -band;
    }

    public static double deadband(double input, double band) {
        return (Math.abs(input) <= band || Math.abs(input) <= -band) ? 0 : input;
    }

    public static double deadband(double input, double band, double setPoint) {
        return (Math.abs(input) <= band || Math.abs(input) <= -band) ? setPoint : input;
    }

    public static boolean poseWithinArea(Pose2d currentPose, Translation2d point1, Translation2d point2) {
        return currentPose.getTranslation().getX() > point1.getX() && currentPose.getTranslation().getX() < point2.getX() &&
                currentPose.getTranslation().getY() > point1.getY() && currentPose.getTranslation().getY() < point2.getY();
    }
}