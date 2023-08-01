package frc.robot.commands.pathfinding.bounds;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public class Rect {
    private final Translation2d blPoint;
    private final Translation2d trPoint;

    public Rect(final Translation2d blPoint, final Translation2d trPoint) {
        this.blPoint = blPoint;
        this.trPoint = trPoint;
    }

    public Rect(final List<Translation2d> points) {
        if (points.size() != 4) {
            throw new IllegalArgumentException("Rect points must be of size 4!");
        }

        final double minX = points.stream().mapToDouble(Translation2d::getX).min().orElseThrow();
        final double maxX = points.stream().mapToDouble(Translation2d::getX).max().orElseThrow();

        final double minY = points.stream().mapToDouble(Translation2d::getY).min().orElseThrow();
        final double maxY = points.stream().mapToDouble(Translation2d::getY).max().orElseThrow();

        this.blPoint = new Translation2d(minX, minY);
        this.trPoint = new Translation2d(maxX, maxY);
    }

    public Rect buffer(final double size) {
        return new Rect(blPoint.plus(new Translation2d(-size, -size)), trPoint.plus(new Translation2d(size, size)));
    }

    public List<Translation2d> getPoints() {
        final Translation2d size = trPoint.minus(blPoint);
        return List.of(
                blPoint,
                blPoint.plus(new Translation2d(0, size.getY())),
                trPoint,
                trPoint.plus(new Translation2d(0, -size.getY()))
        );
    }

    public Translation2d getCentroid() {
        return new Translation2d(
                0.5 * (blPoint.getX() + trPoint.getX()),
                0.5 * (blPoint.getY() + trPoint.getY())
        );
    }

    public boolean contains(final Translation2d point) {
        return point.getX() > blPoint.getX()
                && point.getX() < trPoint.getX()
                && point.getY() > blPoint.getY()
                && point.getY() < trPoint.getY();
    }
}
