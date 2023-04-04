package frc.robot.profiler;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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
                // wanted speed / Teleop max speed
                case FAST:
                    ThrottleWeight = Units.feetToMeters(13) / Constants.Swerve.TELEOP_MAX_SPEED;
                    RotateWeight = Math.PI / 2 /Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED;
                    break;
                case NORMAL:
                    ThrottleWeight = Units.feetToMeters(6) / Constants.Swerve.TELEOP_MAX_SPEED;
                    RotateWeight = Math.PI / 3 /Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED;
                    break;
                case SLOW:
                    ThrottleWeight = Units.feetToMeters(3) / Constants.Swerve.TELEOP_MAX_SPEED;
                    RotateWeight = Math.PI / 4 /Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED;
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