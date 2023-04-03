package frc.robot.profiler;

import frc.robot.utils.Enums;

public class Profiler {
    private static Enums.DriverProfiles profile = Enums.DriverProfiles.DRIVER1;
    private double ThrottleSensitivity = 1;
    private double RotationalSensitivity = 1;

    private static Enums.SwerveSpeeds currentState = Enums.SwerveSpeeds.NORMAL;
    private static double ThrottleWeight = 0.357;
    private static double RotateWeight = 0.5;

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

    public static void setWeights(Enums.SwerveSpeeds state) {
        if (currentState != state) {
            currentState = state;
            switch (state) {
                case FAST:
                    ThrottleWeight = 1;
                    RotateWeight = 0.6;
                    break;
                case NORMAL:
                    ThrottleWeight = 0.357;
                    RotateWeight = 0.5;
                    break;
                case SLOW:
                    ThrottleWeight = 0.214;
                    RotateWeight = 0.2;
                    break;
                default:
                    break;
            }
        }
    }

    public double getThrottleSensitivity() {
        return ThrottleSensitivity;
    }

    public double getRotationalSensitivity() {
        return RotationalSensitivity;
    }

    public double getThrottleWeight() {
        return ThrottleWeight;
    }

    public double getRotateWeight() {
        return RotateWeight;
    }
}