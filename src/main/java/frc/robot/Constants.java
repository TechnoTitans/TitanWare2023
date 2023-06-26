package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.wrappers.api.Slot0Configs;

@SuppressWarnings("unused")
public interface Constants {
    interface Modules {
        double WHEEL_RADIUS = 0.0508; //2 in
        double DRIVER_GEAR_RATIO = 8.14;
        double TURNER_GEAR_RATIO = 150.0 / 7.0;

        double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

        Slot0Configs DRIVE_MOTOR_CONSTANTS = new Slot0Configs(0.00060954, 0.01, 0.25655, 2.9757);
        Slot0Configs TURN_MOTOR_CONSTANTS = new Slot0Configs(30, 0, 0, 0);
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

        //in meters, swerve modules relative to the center of robot
        Translation2d FL_OFFSET = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2); //front left
        Translation2d FR_OFFSET = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2); // front right
        Translation2d BL_OFFSET = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2); // back left
        Translation2d BR_OFFSET = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2); //back right
    }

    interface Vision {
        Transform3d robotToCam = new Transform3d( //x, y, z
                new Translation3d(Units.inchesToMeters(0.5), Units.inchesToMeters(-12.625), Units.inchesToMeters(25)),
                new Rotation3d(0, 0, 0));
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);
        double singleTagMaxAmbiguity = 0.2;
    }

    interface Grid {
        Translation2d LEFTBOTTOM = new Translation2d(1, 3.6);
        Translation2d LEFTTOP = new Translation2d(3.35, 5.26);
        Translation2d CENTERBOTTOM = new Translation2d(1, 1.91);
        Translation2d CENTERTOP = new Translation2d(3.35, 3.58);
        Translation2d RIGHTBOTTOM = new Translation2d(1, 0);
        Translation2d RIGHTTOP = new Translation2d(3.35, 1.89);

        double DISTANCE_FROM_GRID = 1.84;
        double FIELD_LENGTH_X_METERS = 16.54175;
        double FIELD_WIDTH_Y_METERS = 8.0137;

        Pose2d FLIPPING_POSE = new Pose2d(
                new Translation2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS),
                new Rotation2d(Math.PI));

        interface LEFT {
            // 4.98
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 5.045), Rotation2d.fromDegrees(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3), Rotation2d.fromDegrees(180));
            // 3.8
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3.84), Rotation2d.fromDegrees(180));
        }
        interface CENTER {
            // 3.34
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3.38), Rotation2d.fromDegrees(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3), Rotation2d.fromDegrees(180));
            // 2.13
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 2.2), Rotation2d.fromDegrees(180));
        }
        interface RIGHT {
            // 1.6
            Pose2d LEFT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 1.63), Rotation2d.fromDegrees(180));
            Pose2d CUBE = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 3), Rotation2d.fromDegrees(180));
            Pose2d RIGHT = new Pose2d(new Translation2d(DISTANCE_FROM_GRID, 0.47), Rotation2d.fromDegrees(180));
        }
    }
}