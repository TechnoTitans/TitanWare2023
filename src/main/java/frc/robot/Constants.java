package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.wrappers.control.Slot0Configs;

@SuppressWarnings("unused")
public interface Constants {
    RobotMode CURRENT_MODE = RobotMode.SIM;
    CompetitionType CURRENT_COMPETITION_TYPE = CompetitionType.TESTING;
    double LOOP_PERIOD_SECONDS = 0.02;
    double MATCH_END_THRESHOLD_SEC = Units.millisecondsToSeconds(250);

    enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    enum CompetitionType {
        TESTING,
        COMPETITION
    }

    interface CTRE {
        double PHOENIX_5_100ms_PER_SECOND = 10;
        double PHOENIX_5_CANCODER_TICKS_PER_ROTATION = 4096d;
        // apply to CANCoder configuration which makes the sensor return in rotations
        double PHOENIX_5_CANCODER_SENSOR_COEFFICIENT_ROTS = 1d / PHOENIX_5_CANCODER_TICKS_PER_ROTATION;
        String PHOENIX_5_CANCODER_UNIT_STRING_ROTS = "rots";

        boolean DISABLE_NEUTRAL_MODE_IN_SIM = true;
    }

    interface NetworkTables {
        String AUTO_TABLE = "AutoSelector";
        String AUTO_PUBLISHER = "AutoOptions";
        String AUTO_SELECTED_SUBSCRIBER = "SelectedAuto";

        String PROFILE_TABLE = "ProfileSelector";
        String PROFILE_PUBLISHER = "ProfileOptions";
        String PROFILE_SELECTED_SUBSCRIBER = "SelectedProfile";
    }

    interface PathPlanner {
        boolean USE_PATH_PLANNER_SERVER = true;
        int SERVER_PORT = 1683;

        //TODO: it would be better if we could check if we're in a real match
        // and only start the PathPlannerServer if we're not, but DSData delays are annoying to deal with
        // for now, this is addressed by CURRENT_COMPETITION_TYPE
        boolean IS_USING_PATH_PLANNER_SERVER =
                USE_PATH_PLANNER_SERVER
                        && CURRENT_COMPETITION_TYPE != CompetitionType.COMPETITION;
    }

    interface Teleop {
        boolean USE_LEGACY_AUTO_ALIGNMENT = false;
    }

    interface Sim {
        // Assume 2mOhm resistance for voltage drop calculation
        double FALCON_MOTOR_RESISTANCE = 0.002;

        // According to ben (a CTRE intern), adding VelocityTorqueCurrentFOC support is "very low priority"
        // setting this to true configures the swerve to use VelocityVoltage instead when in simulation
        // - keep this true until CTRE adds support for VelocityTorqueCurrentFOC
        boolean USE_VELOCITY_VOLTAGE_IN_SIM = true;

        // Claw Sim
        interface Claw {
            Transform3d CARRIAGE_TO_ROOT_MOUNT_TRANSFORM = new Transform3d(
                    new Translation3d(-0.16, -0.025, 0.105),
                    new Rotation3d(0, Units.degreesToRadians(-110), 0)
            );
            Translation3d SHAFT_TO_CENTER_TRANSLATION = new Translation3d(
                    Units.inchesToMeters(11.75), 0, Units.inchesToMeters(-3.753)
            );
            double OPEN_CLOSE_GEARING = 100;
            double OPEN_CLOSE_MOI = 0.11503541;

            double INTAKE_WHEELS_GEARING = 10;
            double INTAKE_WHEELS_MOI = 0.00079278;

            double TILT_GEARING = 80;
            double TILT_MASS_KG = Units.lbsToKilograms(9.67);
            double TILT_MIN_ANGLE_RAD = Units.degreesToRadians(-10);
            double TILT_MAX_ANGLE_RAD = Units.degreesToRadians(170);
            double TILT_MOI = 1.23814727; // in kg/m^2

            boolean TILT_SIMULATE_GRAVITY = false;

            double CLAW_LENGTH_M = 0.50421;
            double CLAW_HALF_LENGTH_M = CLAW_LENGTH_M * 0.5;
        }


        // Elevator Sim (all length/height units are meters)
        interface Elevator {
            interface Vertical {
                Pose3d ROBOT_TO_ROOT_MOUNT_POSE = new Pose3d(0, 0, 0, new Rotation3d());

                double GEARING = 1 / 0.0938;
                // TODO: find exact (this should be close)
                double MOVING_MASS_KG = Units.lbsToKilograms(30);
                double SPROCKET_RADIUS_M = Units.inchesToMeters(0.7955);
                double SPROCKET_CIRCUMFERENCE_M = 2 * Math.PI * SPROCKET_RADIUS_M;
                double MIN_TOTAL_EXT_M = 0;
                // TODO: find exact (this should be close)
                // TODO: this probably should be calculated from adding the stage one height and its extension height
                double MAX_TOTAL_EXT_M = Units.inchesToMeters(57.52);
                boolean SIMULATE_GRAVITY = true;
                double STAGE_ONE_HEIGHT = 0.83817;
                double STAGE_ONE_EXT_HEIGHT = Units.inchesToMeters(23.75);
                double STAGE_ONE_OFFSET = Units.inchesToMeters(0.5);
                double STAGE_TWO_HEIGHT = 0.20320;
                double STAGE_TWO_EXT_HEIGHT = Units.inchesToMeters(23.75);
                double STAGE_TWO_OFFSET = 0;
                // TODO: find exact?
                double EXT_MOI = 0.1937598419; // moi is in kg/m^2
            }

            interface Horizontal {
                double GEARING = 5;
                double MOVING_MASS_KG = Units.lbsToKilograms(18.7);
                double SPROCKET_RADIUS_M = Units.inchesToMeters(0.57);
                double SPROCKET_CIRCUMFERENCE_M = 2 * Math.PI * SPROCKET_RADIUS_M;
                double MIN_TOTAL_EXT_M = 0;
                double MAX_TOTAL_EXT_M = Units.inchesToMeters(20.1);
                boolean SIMULATE_GRAVITY = false;

                double STAGE_ONE_LENGTH = Units.inchesToMeters(19);
                double STAGE_ONE_EXT_LENGTH = Units.inchesToMeters(12.33);
                double STAGE_ONE_OFFSET = 0;
                double STAGE_TWO_LENGTH = Units.inchesToMeters(19);
                double STAGE_TWO_EXT_LENGTH = Units.inchesToMeters(12.33);
                double STAGE_TWO_OFFSET = 0;
                double EXT_MOI = 4.02; // moi in kg/m^2
            }
        }

        //        Slot0Configs DRIVE_MOTOR_CONSTANTS = new Slot0Configs(0.1, 0, 0, 0.913);
        Slot0Configs DRIVE_MOTOR_CONSTANTS = new Slot0Configs(0, 0, 0, 0.913);
        Slot0Configs TURN_MOTOR_CONSTANTS = new Slot0Configs(40, 0, 0, 0);
    }

    interface Modules {
        double WHEEL_RADIUS = 0.0508; //2 in
        double WHEEL_MASS = 0.2313321; //0.51 lbs
        double DRIVE_WHEEL_MOMENT_OF_INERTIA = WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;
        double TURN_WHEEL_MOMENT_OF_INERTIA = 0.004;
        double DRIVER_GEAR_RATIO = 8.14;
        double TURNER_GEAR_RATIO = 150.0 / 7.0;

        double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

        //TODO: tune for no kV
        Slot0Configs DRIVE_MOTOR_CONSTANTS = new Slot0Configs(50, 0, 0, 2);
        Slot0Configs TURN_MOTOR_CONSTANTS = new Slot0Configs(30, 0, 0.5, 0);
    }

    interface Swerve {
        double WHEEL_BASE = 0.7366;
        double TRACK_WIDTH = 0.7366;
        double MAX_WIDTH = Math.max(WHEEL_BASE, TRACK_WIDTH);
        double HALF_MAX_WIDTH = 0.5 * MAX_WIDTH;
        double ROBOT_MAX_SPEED = Units.feetToMeters(13.5);
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
        //L = Left, R = Right, F = Forward, B = Backward (Facing)
        Transform3d ROBOT_TO_FR_APRILTAG_CAM_R = new Transform3d(
                new Translation3d(Units.inchesToMeters(13.449), Units.inchesToMeters(-13.762), Units.inchesToMeters(7.922)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-70))
        );

        Transform3d ROBOT_TO_FR_APRILTAG_CAM_F = new Transform3d(
                new Translation3d(Units.inchesToMeters(14.465), Units.inchesToMeters(-11.907), Units.inchesToMeters(9.67)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(25))
        );

        Transform3d ROBOT_TO_FL_APRILTAG_CAM_L = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.78474), Units.inchesToMeters(13.52291), Units.inchesToMeters(9.43904)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(120))
        );

        Transform3d ROBOT_TO_BR_APRILTAG_CAM_B = new Transform3d(
                new Translation3d(Units.inchesToMeters(-11.87298), Units.inchesToMeters(-11.36695), Units.inchesToMeters(9.151)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(205))
        );

        /**
         * Standard deviations of the supplied pose estimate (before vision, likely to be solely wheel odometry)
         */
        Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
        Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(1, 1, 1);
        double MULTI_TAG_MAX_AMBIGUITY = 0.3;
        double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
    }
}