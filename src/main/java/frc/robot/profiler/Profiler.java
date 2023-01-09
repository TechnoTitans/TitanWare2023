package frc.robot.profiler;

public class Profiler {
    private static Profile profile = null;

    public static void setProfile(Profile p) {
        profile = p;
    }

    public static Profile getProfile() {
        return profile;
    }
}