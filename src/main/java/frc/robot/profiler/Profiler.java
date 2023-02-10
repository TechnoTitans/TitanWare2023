package frc.robot.profiler;

public class Profiler {
    public enum Profiles {
        Driver1,
        Driver2
    }

    private static Profiles profile;
    public double ThrottleSensitivity = 1;
    public double SpinningSensitivity = 1;

    private Profiler() {}

    public double getThrottleSensitivity() {
        return ThrottleSensitivity;
    }

    public double getSpinningSensitivity() {
        return SpinningSensitivity;
    }

    public static void setProfile(Profiles profile) {
        Profiler.profile = profile;
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
}