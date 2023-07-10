package frc.robot.utils.pathplanner;

import frc.robot.Constants;

public record AutoOption(String pathName, double maxVelocity, double maxAcceleration) {
    public AutoOption(final String pathName) {
        this(pathName, Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION);
    }
}
