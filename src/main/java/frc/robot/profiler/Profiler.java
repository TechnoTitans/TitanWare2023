package frc.robot.profiler;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Profiler {
    private static DriverProfile driverProfile = DriverProfile.DEFAULT;
    private static SwerveSpeed swerveSpeed = SwerveSpeed.NORMAL;

    private Profiler() {}

    public static DriverProfile getDriverProfile() {
        return driverProfile;
    }

    public static void setDriverProfile(final DriverProfile driverProfile) {
        if (driverProfile != null) {
            Profiler.driverProfile = driverProfile;
        }
    }

    public static SwerveSpeed getSwerveSpeed() {
        return swerveSpeed;
    }

    public static void setSwerveSpeed(final SwerveSpeed swerveSpeed) {
        Profiler.swerveSpeed = swerveSpeed;
    }

    public enum DriverProfile {
        DRIVER1(1, 1),
        DRIVER2(1.1, 1.1),
        DEFAULT(1, 1);

        final double throttleSensitivity;
        final double rotationalSensitivity;

        DriverProfile(final double throttleSensitivity, final double rotationalSensitivity) {
            this.throttleSensitivity = throttleSensitivity;
            this.rotationalSensitivity = rotationalSensitivity;
        }

        public double getThrottleSensitivity() {
            return throttleSensitivity;
        }

        public double getRotationalSensitivity() {
            return rotationalSensitivity;
        }
    }

    public enum SwerveSpeed {
        //Takes feet and radians as scalar values
        FAST(Units.feetToMeters(13), 0.5),
        NORMAL(Units.feetToMeters(6), 0.35),
        SLOW(Units.feetToMeters(2), 0.25);

        final double throttleWeight;
        final double rotateWeight;

        SwerveSpeed(final double throttleWeight, final double rotateWeight) {
            this.throttleWeight = throttleWeight / Constants.Swerve.TELEOP_MAX_SPEED;
            this.rotateWeight = (Math.PI * rotateWeight) / Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED;
        }

        public double getThrottleWeight() {
            return throttleWeight;
        }

        public double getRotateWeight() {
            return rotateWeight;
        }
    }
}
