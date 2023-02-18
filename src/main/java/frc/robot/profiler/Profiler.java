package frc.robot.profiler;

import frc.robot.utils.Enums;

public class Profiler {
    private static Enums.DriverProfiles profile;
    private double ThrottleSensitivity = 1;
    private double SpinningSensitivity = 1;

    private Profiler() {
    }

    public static Profiler getProfile() {
        Profiler newProfile = new Profiler();
        switch (profile) {
            case Driver1:
                newProfile.SpinningSensitivity = 0.9;
                newProfile.ThrottleSensitivity = 0.9;
                break;
            case Driver2:
                newProfile.SpinningSensitivity = 1.1;
                newProfile.ThrottleSensitivity = 1.1;
                break;
            default:
                newProfile.SpinningSensitivity = 1;
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

    public double getSpinningSensitivity() {
        return SpinningSensitivity;
    }
}