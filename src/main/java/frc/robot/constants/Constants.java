package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator;

public interface Constants {
    //We have yet to use replay in a useful way and this is much easier to deal with
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

    interface PDH {
        double BATTERY_NOMINAL_VOLTAGE = 12;
        double BATTERY_RESISTANCE_OHMS = 0.02;

        int[] DRIVETRAIN_CHANNELS = {3, 4, 6, 7, 19, 16, 18, 17};
        int[] VERTICAL_ELEVATOR_CHANNELS = {8, 5};
        int[] HORIZONTAL_ELEVATOR_CHANNELS = {10};
        int[] CLAW_CHANNELS = {11, 0, 1, 2};
    }

    interface NetworkTables {
        String AUTO_TABLE = "AutoSelector";
        String AUTO_PUBLISHER = "AutoOptions";
        String AUTO_SELECTED_SUBSCRIBER = "SelectedAuto";

        String PROFILE_TABLE = "ProfileSelector";
        String PROFILE_PUBLISHER = "ProfileOptions";
        String PROFILE_SELECTED_SUBSCRIBER = "SelectedProfile";

        boolean USE_STRUCT_AND_PROTOBUF = true;
    }

    interface Teleop {
        boolean USE_LEGACY_AUTO_ALIGNMENT = false;
    }

    interface Swerve {
        double WHEEL_BASE = 0.7366;
        double TRACK_WIDTH = 0.7366;
        double ROBOT_MAX_SPEED = Units.feetToMeters(13);
        double ROBOT_MAX_ANGULAR_SPEED = 2 * Math.PI;
        double TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        double TELEOP_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_SPEED = 2;
        double TRAJECTORY_MAX_ACCELERATION = 2;
        double TRAJECTORY_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJECTORY_MAX_ANGULAR_ACCELERATION = 1.5 * ROBOT_MAX_ANGULAR_SPEED;

        interface Modules {
            double WHEEL_RADIUS = 0.0508; //2 in
            double WHEEL_MASS = 0.2313321; //0.51 lbs
            double DRIVE_WHEEL_MOMENT_OF_INERTIA = WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;
            double TURN_WHEEL_MOMENT_OF_INERTIA = 0.004;
            double DRIVER_GEAR_RATIO = 8.14;
            double TURNER_GEAR_RATIO = 150.0 / 7.0;

            double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
            // SDS MK4i L1 with Falcon 500 FOC
            // see https://www.swervedrivespecialties.com/products/mk4i-swerve-module
            double MODULE_MAX_SPEED = Units.feetToMeters(13);
        }
    }

    interface Vision {
        PhotonPoseEstimator.PoseStrategy MULTI_TAG_POSE_STRATEGY =
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

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
                new Translation3d(Units.inchesToMeters(11.93), Units.inchesToMeters(12.45), Units.inchesToMeters(9.4)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(55))
        );

        Transform3d ROBOT_TO_BR_APRILTAG_CAM_B = new Transform3d(
                new Translation3d(Units.inchesToMeters(-11.78), Units.inchesToMeters(-11.22), Units.inchesToMeters(10.17)),
                new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(155.05))
        );

        /**
         * Standard deviations of the supplied pose estimate (before vision, likely to be solely wheel odometry)
         */
        Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(2.5));
        Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.85, 0.85, Units.degreesToRadians(5));
        double MULTI_TAG_MAX_AMBIGUITY = 0.3;
        double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
    }
}
