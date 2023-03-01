package frc.robot.profiler;

import frc.robot.utils.Enums;

public class Profiler {
    private static Enums.DriverProfiles profile;
    private double ThrottleSensitivity = 1;
    private double RotationalSensitivity = 1;

    private Profiler() {
    }

    public static Profiler getProfile() {
        Profiler newProfile = new Profiler();
        switch (profile) {
            case DRIVER1:
                newProfile.RotationalSensitivity = 1;
                newProfile.ThrottleSensitivity = 1;
                break;
            case DRIVER2:
                newProfile.RotationalSensitivity = 1.1;
                newProfile.ThrottleSensitivity = 1.1;
                break;
            default:
                newProfile.RotationalSensitivity = 1;
                newProfile.ThrottleSensitivity = 1;
                break;
        }
        return newProfile;
    }

    public static void setProfile(Enums.DriverProfiles profile) {
        Profiler.profile = profile;
    }

    public double getThrottleSensitivity() {
        return ThrottleSensitivity;
    }

    public double getRotationalSensitivity() {
        return RotationalSensitivity;
    }
}