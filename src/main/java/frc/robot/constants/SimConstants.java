package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface SimConstants {
    interface Rev {
        boolean SIM_USE_ROBORIO_PID_FOR_POSITION = true;
    }

    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

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
        // TODO: something's wrong with the min angle here
        double TILT_MIN_ANGLE_RAD = Units.degreesToRadians(-30);
        double TILT_MAX_ANGLE_RAD = Units.degreesToRadians(170);
        double TILT_MOI = 1.23814727; // in kg/m^2

        boolean TILT_SIMULATE_GRAVITY = false;

        double CLAW_LENGTH_M = 0.50421;
    }

    /**
     * Elevator Sim (all length/height units are meters)
     */
    interface Elevator {
        interface Vertical {
            Pose3d ROBOT_TO_ROOT_MOUNT_POSE = new Pose3d();

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
}
