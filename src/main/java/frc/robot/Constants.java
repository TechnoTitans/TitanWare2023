package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.wrappers.api.Slot0Configs;

@SuppressWarnings("unused")
public interface Constants {
    RobotMode CURRENT_MODE = RobotMode.SIM;
    double LOOP_PERIOD_SECONDS = 0.02;

    enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    interface CTRE {
        double PHOENIX_5_100ms_PER_SECOND = 10;
        double PHOENIX_5_CANCODER_TICKS_PER_ROTATION = 4096d;
        // apply to CANCoder configuration which makes the sensor return in rotations
        double PHOENIX_5_CANCODER_SENSOR_COEFFICIENT_ROTS = 1d / PHOENIX_5_CANCODER_TICKS_PER_ROTATION;
        String PHOENIX_5_CANCODER_UNIT_STRING_ROTS = "rots";
    }

    interface PathPlanner {
        boolean USE_PATH_PLANNER_SERVER = true;
        int SERVER_PORT = 1683;

        //TODO: it would be better if we could check if we're in a real match
        // and only start the PathPlannerServer if we're not, but DSData delays are annoying to deal with
        boolean IS_USING_PATH_PLANNER_SERVER = USE_PATH_PLANNER_SERVER && CURRENT_MODE == RobotMode.SIM;
    }

    interface Sim {
        // Assume 2mOhm resistance for voltage drop calculation
        double FALCON_MOTOR_RESISTANCE = 0.002;

        // According to ben (that one CTRE intern), adding VelocityTorqueCurrentFOC support is "very low priority"
        // setting this to true configures the swerve to use VelocityVoltage instead when in simulation
        // - keep this true until CTRE adds support for VelocityTorqueCurrentFOC
        boolean USE_VELOCITY_VOLTAGE_IN_SIM = true;


        // Elevator Sim (all length/height units are meters)
        double ELEVATOR_VERTICAL_STAGE_ONE_HEIGHT = 0.8635746;
        double ELEVATOR_VERTICAL_STAGE_ONE_EXT_HEIGHT = 0.6096;
        double ELEVATOR_VERTICAL_STAGE_TWO_HEIGHT = 0.2286;
        double ELEVATOR_VERTICAL_STAGE_TWO_EXT_HEIGHT = 0.5706364;
        double ELEVATOR_HORIZONTAL_STAGE_ONE_LENGTH = 0.508;
        double ELEVATOR_HORIZONTAL_STAGE_TWO_LENGTH = 0.508;

        double ELEVATOR_VERTICAL_EXT_MOI = 0.1937598419; // moi is in kg/m^2

        Slot0Configs DRIVE_MOTOR_CONSTANTS = new Slot0Configs(0.8, 0, 0, 0.97);
        Slot0Configs TURN_MOTOR_CONSTANTS = new Slot0Configs(20, 0, 1, 0);
    }

    interface Modules {
        double WHEEL_RADIUS = 0.0508; //2 in
        double WHEEL_MASS = 0.2313321; //0.51 lbs
        double DRIVE_WHEEL_MOMENT_OF_INERTIA = WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;
        double TURN_WHEEL_MOMENT_OF_INERTIA = 0.004;
        double DRIVER_GEAR_RATIO = 8.14;
        double TURNER_GEAR_RATIO = 150.0 / 7.0;

        double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

        Slot0Configs DRIVE_MOTOR_CONSTANTS = new Slot0Configs(50, 0, 0, 2);
        Slot0Configs TURN_MOTOR_CONSTANTS = new Slot0Configs(30, 0, 0.5, 0);
    }

    interface Swerve {
        double WHEEL_BASE = 0.7366;
        double TRACK_WIDTH = 0.7366;
        double ROBOT_MAX_SPEED = Units.feetToMeters(13);
        double MODULE_MAX_SPEED = Units.feetToMeters(13.5);
        double ROBOT_MAX_ANGULAR_SPEED = Math.PI;
        double TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        double TELEOP_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_SPEED = 4;
        double TRAJECTORY_MAX_ACCELERATION = 4;
        double TRAJECTORY_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_ANGULAR_ACCELERATION = Math.PI;
        double ROTATE_P = 1;

        boolean USE_SWERVE_SKEW_FIX = false;

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
        Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);
        double singleTagMaxAmbiguity = 0.2;
    }

    interface Field {
        Translation2d LEFT_BOTTOM = new Translation2d(1, 3.6);
        Translation2d LEFT_TOP = new Translation2d(3.35, 5.26);
        Translation2d CENTER_BOTTOM = new Translation2d(1, 1.91);
        Translation2d CENTER_TOP = new Translation2d(3.35, 3.58);
        Translation2d RIGHT_BOTTOM = new Translation2d(1, 0);
        Translation2d RIGHT_TOP = new Translation2d(3.35, 1.89);

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