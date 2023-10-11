package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public interface Constants {
    RobotHardware ROBOT_HARDWARE = RobotHardware.ROBOT_2023_FALCON_SWERVE;
    RobotMode CURRENT_MODE = RobotMode.REAL;
    CompetitionType CURRENT_COMPETITION_TYPE = CompetitionType.TESTING;
    double LOOP_PERIOD_SECONDS = 0.02;
    double MATCH_END_THRESHOLD_SEC = Units.millisecondsToSeconds(250);

    enum RobotHardware {
        ROBOT_2023_FALCON_SWERVE,
        ROBOT_2023_NEO_SWERVE
    }

    enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    enum CompetitionType {
        TESTING,
        COMPETITION
    }

    enum ClosedLoopControllerType {
        PID,
        STATE_SPACE
    }

    interface CTRE {
        double PHOENIX_5_100ms_PER_SECOND = 10;
        double PHOENIX_5_CANCODER_TICKS_PER_ROTATION = 4096d;
        // apply to CANCoder configuration which makes the sensor return in rotations
        double PHOENIX_5_CANCODER_SENSOR_COEFFICIENT_ROTS = 1d / PHOENIX_5_CANCODER_TICKS_PER_ROTATION;
        String PHOENIX_5_CANCODER_UNIT_STRING_ROTS = "rots";

        boolean DISABLE_NEUTRAL_MODE_IN_SIM = true;
    }

    interface PDH {
        double BATTERY_NOMINAL_VOLTAGE = 12;
        double BATTERY_RESISTANCE_OHMS = 0.02;

        int[] DRIVETRAIN_CHANNELS = {0, 1, 2, 3, 4, 5, 6, 7};
        int[] VERTICAL_ELEVATOR_CHANNELS = {10, 11};
        int[] HORIZONTAL_ELEVATOR_CHANNELS = {12};
        int[] CLAW_CHANNELS = {13, 14, 15, 16};
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
        boolean USE_PATH_PLANNER_SERVER = false;
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

    interface Modules {
        double WHEEL_RADIUS = 0.0508; //2 in
        double WHEEL_MASS = 0.2313321; //0.51 lbs
        double DRIVE_WHEEL_MOMENT_OF_INERTIA = WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;
        double TURN_WHEEL_MOMENT_OF_INERTIA = 0.004;
        double DRIVER_GEAR_RATIO = 8.14;
        double TURNER_GEAR_RATIO = 150.0 / 7.0;

        double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    }

    interface Swerve {
        double WHEEL_BASE = 0.7366;
        double TRACK_WIDTH = 0.7366;
        double ROBOT_MAX_SPEED = Units.feetToMeters(13.5);
        double MODULE_MAX_SPEED = Units.feetToMeters(13.5);
        double ROBOT_MAX_ANGULAR_SPEED = 2 * Math.PI;
        double TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        double TELEOP_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_SPEED = 4;
        double TRAJECTORY_MAX_ACCELERATION = 4;
        double TRAJECTORY_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_ANGULAR_ACCELERATION = 1.5 * ROBOT_MAX_ANGULAR_SPEED;
        double ROTATE_P = 1;

        boolean USE_SWERVE_SKEW_FIX = true;

        //in meters, swerve modules relative to the center of robot
        Translation2d FL_OFFSET = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2); //front left
        Translation2d FR_OFFSET = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2); // front right
        Translation2d BL_OFFSET = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2); // back left
        Translation2d BR_OFFSET = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2); //back right
    }

    interface Claw {
        ClosedLoopControllerType CONTROLLER = ClosedLoopControllerType.STATE_SPACE;
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
