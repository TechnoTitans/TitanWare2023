package frc.robot.profiler;

public class Profiler {
    public enum Profiles {
        Driver1,
        Driver2
    }

    private static Profiles profile;
    private final double ThrottleSensitivity;
    private final double SpinningSensitivity;

    private Profiler(double ThrottleSensitivity, double SpinningSensitivity) {
        this.ThrottleSensitivity = ThrottleSensitivity;
        this.SpinningSensitivity = SpinningSensitivity;
    }

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
        switch (profile) {
            case Driver1:
                return new Profiler(1, 1);
            case Driver2:
                return new Profiler(1.1, 1);
            default:
                return new Profiler(1, 1);
        }
    }
}