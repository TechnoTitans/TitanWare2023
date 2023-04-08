package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

@SuppressWarnings("unused")
public interface Constants {
    interface Modules {
        double WHEEL_RADIUS = 0.0508; //2 in
        int MOTOR_ROTATION_TO_TALON_ENCODER_TICKS = 2048;
        int CANCODER_TICKS_PER_ROTATION = 4096;
        double DRIVER_GEAR_RATIO = 8.14;
        double TURNER_GEAR_RATIO = 150.0 / 7.0;
        double TICKS_PER_MOTOR_RADIAN = MOTOR_ROTATION_TO_TALON_ENCODER_TICKS / (2 * Math.PI);
        double TICKS_PER_CANCODER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360.0;
    }

    interface Swerve {
        double WHEEL_BASE = 0.7366;
        double TRACK_WIDTH = 0.7366;
        double ROBOT_MAX_SPEED = Units.feetToMeters(13);
        double MODULE_MAX_SPEED = Units.feetToMeters(13.5);
        double ROBOT_MAX_ANGULAR_SPEED = Math.PI;
        double TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        double TELEOP_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJ_MAX_SPEED = 4;
        double TRAJ_MAX_ACCELERATION = 3;
        double TRAJ_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJ_MAX_ANGULAR_ACCELERATION = Math.PI;
        double ROTATE_P = 1;
    }

    interface Grid {
        Translation2d LEFTBOTTOM = new Translation2d(1, 3.6);
        Translation2d LEFTTOP = new Translation2d(3.35, 5.26);
        Translation2d CENTERBOTTOM = new Translation2d(1, 1.91);
        Translation2d CENTERTOP = new Translation2d(3.35, 3.58);
        Translation2d RIGHTBOTTOM = new Translation2d(1, 0);
        Translation2d RIGHTTOP = new Translation2d(3.35, 1.89);

        double DISTANCE_FROM_GRID = 1.83;
        double FIELD_WIDTH_METERS = 8.02;

        interface LEFT {
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 4.98), Rotation2d.fromDegrees(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3), Rotation2d.fromDegrees(180));
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3.8), Rotation2d.fromDegrees(180));
        }
        interface CENTER {
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3.34), Rotation2d.fromDegrees(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3), Rotation2d.fromDegrees(180));
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 2.13), Rotation2d.fromDegrees(180));
        }
        interface RIGHT {
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 1.6), Rotation2d.fromDegrees(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3), Rotation2d.fromDegrees(180));
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 0.47), Rotation2d.fromDegrees(180));
        }
    }
}