package frc.robot.profiler;

import frc.robot.utils.Enums;

public class Profiler {
    private static Enums.DriverProfile driverProfile = Enums.DriverProfile.DEFAULT;
    private static Enums.SwerveSpeed swerveSpeed = Enums.SwerveSpeed.NORMAL;

    private Profiler() {}

    public static Enums.DriverProfile getDriverProfile() {
        return driverProfile;
    }

    public static void setDriverProfile(final Enums.DriverProfile driverProfile) {
        if (driverProfile != null) {
            Profiler.driverProfile = driverProfile;
        }
    }

    public static Enums.SwerveSpeed getSwerveSpeed() {
        return swerveSpeed;
    }

    public static void setSwerveSpeed(final Enums.SwerveSpeed swerveSpeed) {
        Profiler.swerveSpeed = swerveSpeed;
    }
}
