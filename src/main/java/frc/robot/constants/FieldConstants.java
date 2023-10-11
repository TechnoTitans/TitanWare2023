package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final double GRID_SCORING_X_POSITION = 1.84;
    public static final double SUBSTATION_PICKUP_X_POSITION = 15.8;
    public static final double FIELD_LENGTH_X_METERS = 16.54175;
    public static final double FIELD_WIDTH_Y_METERS = 8.0137;
    public static final Pose2d FLIPPING_POSE = new Pose2d(
            new Translation2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS),
            new Rotation2d(Math.PI));
    public static final double LOADING_ZONE_WIDTH_Y_METERS = 2.52;
    public static final double FIELD_WITHIN_BORDER_MARGIN = 0.5;
    public static final double FIELD_WITHIN_Z_MARGIN = 0.75;
}
