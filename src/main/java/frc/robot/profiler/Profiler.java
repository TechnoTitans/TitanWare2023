package frc.robot.profiler;

import frc.robot.utils.Enums;

public class Profiler {
    private static Enums.DriverProfiles profile = Enums.DriverProfiles.DRIVER1;
    private double ThrottleSensitivity = 1;
    private double RotationalSensitivity = 1;

    private double ThrottleFastWeight = 1;
    private double RotateFastWeight = 0.6;

    private double ThrottleNormalWeight = 0.357; //.357
    private double RotateNormalWeight = 0.5; //.7

    private double ThrottleSlowWeight = 0.214;
    private double RotateSlowWeight = 0.357;

    private Profiler() {}

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

    public double getThrottleFastWeight() {
        return ThrottleFastWeight;
    }

    public double getRotateFastWeight() {
        return RotateFastWeight;
    }

    public double getThrottleNormalWeight() {
        return ThrottleNormalWeight;
    }

    public double getRotateNormalWeight() {
        return RotateNormalWeight;
    }

    public double getThrottleSlowWeight() {
        return ThrottleSlowWeight;
    }

    public double getRotateSlowWeight() {
        return RotateSlowWeight;
    }
}