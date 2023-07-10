package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

@SuppressWarnings("unused")
public class MathUtils {

    // Method for checking if a value is within a specific range using threshold
    public static boolean withinRange(
            final double input,
            final double center,
            final double range
    ) {
        return input > (center - range) && input < (center + range);
    }

    // Method for checking if a value is within a specific range using 2 thresholds
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

    public static double doubleLerp(double startVal, double endVal, double t) {
        return startVal + (endVal - startVal) * t;
    }

    public static Rotation2d rotationLerp(Rotation2d startVal, Rotation2d endVal, double t) {
        return startVal.plus(endVal.minus(startVal).times(t));
    }

    public static Translation2d translationLerp(Translation2d a, Translation2d b, double t) {
        return a.plus((b.minus(a)).times(t));
    }

    public static Translation2d quadraticLerp(Translation2d a, Translation2d b, Translation2d c, double t) {
        Translation2d p0 = translationLerp(a, b, t);
        Translation2d p1 = translationLerp(b, c, t);
        return translationLerp(p0, p1, t);
    }

    public static Translation2d cubicLerp(
            Translation2d a, Translation2d b, Translation2d c, Translation2d d, double t) {
        Translation2d p0 = quadraticLerp(a, b, c, t);
        Translation2d p1 = quadraticLerp(b, c, d, t);
        return translationLerp(p0, p1, t);
    }

    public static Rotation2d cosineInterpolate(Rotation2d y1, Rotation2d y2, double mu) {
        double mu2 = (1 - Math.cos(mu * Math.PI)) / 2;
        return new Rotation2d(y1.getRadians() * (1 - mu2) + y2.getRadians() * mu2);
    }
}