package frc.robot.profiler;

import frc.robot.utils.SuperstructureStates;

public class Profiler {
    private static SuperstructureStates.DriverProfile driverProfile = SuperstructureStates.DriverProfile.DEFAULT;
    private static SuperstructureStates.SwerveSpeed swerveSpeed = SuperstructureStates.SwerveSpeed.NORMAL;

    private Profiler() {}

    public static SuperstructureStates.DriverProfile getDriverProfile() {
        return driverProfile;
    }

    public static void setDriverProfile(final SuperstructureStates.DriverProfile driverProfile) {
        if (driverProfile != null) {
            Profiler.driverProfile = driverProfile;
        }
    }

    public static SuperstructureStates.SwerveSpeed getSwerveSpeed() {
        return swerveSpeed;
    }

    public static void setSwerveSpeed(final SuperstructureStates.SwerveSpeed swerveSpeed) {
        Profiler.swerveSpeed = swerveSpeed;
    }
}
