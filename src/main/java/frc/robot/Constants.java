package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

//@SuppressWarnings("unused")
public interface Constants {
    interface Modules {
        double WHEEL_RADIUS = 0.0508; //2 in
        int MOTOR_ROTATION_TO_TALON_ENCODER_TICKS = 2048;
        int CANCODER_TICKS_PER_ROTATION = 4096;
        double DRIVER_GEAR_RATIO = 8.14;
        double TURNER_GEAR_RATIO = 150.0 / 7.0;
        double TICKS_PER_MOTOR_RADIAN = MOTOR_ROTATION_TO_TALON_ENCODER_TICKS / (2 * Math.PI);
        double DRIVER_TICKS_PER_WHEEL_RADIAN = TICKS_PER_MOTOR_RADIAN * DRIVER_GEAR_RATIO;
        double TICKS_PER_DRIVER_WHEEL_ROTATION = MOTOR_ROTATION_TO_TALON_ENCODER_TICKS * DRIVER_GEAR_RATIO;
        int HUNDREDMILLISECONDS_TO_1SECOND = 10;
        double ONESECOND_TO_100_MILLISECONDS = .1;
        double TICKS_PER_TALON_ENCODER_DEGREE = MOTOR_ROTATION_TO_TALON_ENCODER_TICKS / 360.0;
        double TICKS_PER_CANCODER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360.0;
    }

    interface Swerve {
        double WHEEL_BASE = 0.7366;
        double TRACK_WIDTH = 0.7366;
        double ROBOT_MAX_SPEED = Units.feetToMeters(9);
        double MODULE_MAX_SPEED = Units.feetToMeters(13.5);
        double ROBOT_MAX_ANGULAR_SPEED = Math.PI;
        double TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        double TELEOP_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJ_MAX_SPEED = 4;
        double TRAJ_MAX_ACCELERATION = 3;
        double TRAJ_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJ_MAX_ANGULAR_ACCELERATION = Math.PI;
        double ROTATE_P = 1; //TUNE THIS: (rotation pid) 2
        double AUTO_BALANCE_PITCH_P = 0.005; // P value for auto balance
    }

    interface Grid {
        Translation2d LEFTBOTTOM = new Translation2d(1.81, 3.64);
        Translation2d LEFTTOP = new Translation2d(3.15, 5.26);
        Translation2d CENTERBOTTOM = new Translation2d(2.73, 3.45);
        Translation2d CENTERTOP = new Translation2d(1.77, 1.92);
        Translation2d RIGHTBOTTOM = new Translation2d(1.64, 0.15);
        Translation2d RIGHTTOP = new Translation2d(2.73, 1.83);

        double DISTANCE_FROM_GRID = 1.83;

        interface LEFT {
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 4.93), new Rotation2d(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 4.43), new Rotation2d(180));
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3.94), new Rotation2d(180));
        }
        interface CENTER {
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3.27), new Rotation2d(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 2.74), new Rotation2d(180));
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 2.14), new Rotation2d(180));
        }
        interface RIGHT {
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 1.59), new Rotation2d(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 1.06), new Rotation2d(180));
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 0.5), new Rotation2d(180));
        }
    }
}