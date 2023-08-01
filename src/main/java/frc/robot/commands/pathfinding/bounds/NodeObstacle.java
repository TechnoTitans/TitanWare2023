package frc.robot.commands.pathfinding.bounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class NodeObstacle {
    private final Boundary boundary;

    public NodeObstacle(final List<Translation2d> points, final double bufferDistance) {
        final Boundary unBufferedBoundary = new Boundary(points);
        final Translation2d unscaledCentroid = unBufferedBoundary.getCentroid();

        this.boundary = unBufferedBoundary.buffer(bufferDistance);

        Logger.getInstance().recordOutput(
                "Center",
                new Pose2d(unscaledCentroid, new Rotation2d())
        );
        Logger.getInstance().recordOutput(
                "Unscaled",
                unBufferedBoundary.asPose2ds().toArray(Pose2d[]::new)
        );
        Logger.getInstance().recordOutput(
                "Boundary",
                boundary.asPose2ds().toArray(Pose2d[]::new)
        );
    }

    public boolean isObstructed(final Translation2d point) {
        return boundary.contains(point);
    }
}
