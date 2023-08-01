package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.pathfinding.bounds.NodeObstacle;

import java.util.List;

public class FieldConstants {
    public static final double HALF_TRACK_WIDTH =
            Math.max(Constants.Swerve.TRACK_WIDTH, Constants.Swerve.WHEEL_BASE) * 0.5;

    public static final int NODE_DENSITY = 5;
    public static final double GRID_SCORING_X_POSITION = 1.84;
    public static final double SUBSTATION_PICKUP_X_POSITION = 15.8;
    public static final double FIELD_LENGTH_X_METERS = 16.54175;
    public static final double FIELD_WIDTH_Y_METERS = 8.0137;
    public static final Pose2d FLIPPING_POSE = new Pose2d(
            new Translation2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS),
            new Rotation2d(Math.PI));
    public static final double LOADING_ZONE_WIDTH_Y_METERS = 2.52;
    public static final double BARRIER_WIDTH_Y_METERS = 0.1985;

    public static class BlueObstacles {
        public static final NodeObstacle CHARGE_STATION = new NodeObstacle(
                List.of(
                        new Translation2d(2.92, 3.96),
                        new Translation2d(4.85, 3.96),
                        new Translation2d(4.85, 1.52),
                        new Translation2d(2.92, 1.52)
                ),
                HALF_TRACK_WIDTH
        );
        public static final NodeObstacle LOADING_ZONE = new NodeObstacle(
                List.of(
                        new Translation2d(9.86, 8),
                        new Translation2d(9.86, 6.79),
                        new Translation2d(13.18, 6.79),
                        new Translation2d(13.18, 5.46),
                        new Translation2d(16.48, 5.46),
                        new Translation2d(16.48, 8)
                ),
                0
        );
        public static final NodeObstacle GRID = new NodeObstacle(
                List.of(
                        new Translation2d(1.38, 5.46),
                        new Translation2d(1.38, 0),
                        new Translation2d(0, 0),
                        new Translation2d(0, 5.46)
                ),
                HALF_TRACK_WIDTH
        );
        public static final NodeObstacle BARRIER = new NodeObstacle(
                List.of(
                        new Translation2d(1.37, 5.47),
                        new Translation2d(3.39, 5.47),
                        new Translation2d(3.39, 5.55),
                        new Translation2d(1.37, 5.55)
                ),
                HALF_TRACK_WIDTH
        );
        public static final NodeObstacle DOUBLE_SUBSTATION = new NodeObstacle(
                List.of(
                        new Translation2d(16.23, 8),
                        new Translation2d(16.23, 5.5),
                        new Translation2d(16.53, 5.5),
                        new Translation2d(16.53, 8)
                ),
                HALF_TRACK_WIDTH
        );

        public static List<NodeObstacle> getAll() {
            return List.of(
                    CHARGE_STATION,
                    LOADING_ZONE,
                    GRID,
                    BARRIER,
                    DOUBLE_SUBSTATION
            );
        }
    }
}
