package frc.robot.commands.pathfinding.bounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class Boundary {
    private final List<Translation2d> points;
    private final boolean isRect;
    private final Rect rect;

    public Boundary(final List<Translation2d> points) {
        this.points = points;
        this.isRect = points.size() == 4;
        this.rect = isRect ? new Rect(points) : null;
    }

    public Boundary(final Rect rect) {
        this.points = rect.getPoints();
        this.isRect = true;
        this.rect = rect;
    }

    public List<Pose2d> asPose2ds() {
        return points.stream().map(translation2d -> new Pose2d(translation2d, new Rotation2d())).toList();
    }

    public Boundary buffer(final double size) {
        if (isRect) {
            return new Boundary(rect.buffer(size));
        }

        final List<Translation2d> bufferedPoints = new ArrayList<>();
        final Translation2d unBufferedCentroid = getCentroid();

        for (final Translation2d point : points) {
            final Translation2d relative = point.minus(unBufferedCentroid);
            final Translation2d bufferedPoint = point
                    .minus(unBufferedCentroid)
                    .times(1 +
                            (size / Math.hypot(relative.getX(), relative.getY()))
                    )
                    .plus(unBufferedCentroid);

            bufferedPoints.add(bufferedPoint);
        }

        return new Boundary(bufferedPoints);
    }

    /**
     * Get the Centroid of the polygon described by this {@link Boundary}.
     *
     * <p>The Centroid is either computed from the {@link Rect} if {@link Boundary#isRect} is true,
     * else, it is computed using <a href="https://en.wikipedia.org/wiki/Centroid#Of_a_polygon">this algorithm</a></p>
     *
     * @return the Centroid, as a {@link Translation2d}
     */
    public Translation2d getCentroid() {
        if (isRect) {
            // fast path if rect
            return rect.getCentroid();
        }

        // from https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
        final double area = 0.5 *
                IntStream.range(0, points.size())
                        .mapToDouble(i -> {
                            final Translation2d point = points.get(i);
                            final Translation2d nextPoint = (i + 1) >= points.size()
                                    ? points.get(0)
                                    : points.get(i + 1);

                            return (point.getX() * nextPoint.getY()) - (nextPoint.getX() * point.getY());
                        })
                        .sum();

        final double k = 1d / (6 * area);
        final double cX = k *
                IntStream.range(0, points.size())
                        .mapToDouble(i -> {
                            final Translation2d point = points.get(i);
                            final Translation2d nextPoint = (i + 1) >= points.size()
                                    ? points.get(0)
                                    : points.get(i + 1);

                            return (point.getX() + nextPoint.getX())
                                            * ((point.getX() * nextPoint.getY()) - (nextPoint.getX() * point.getY()));
                        })
                        .sum();

        final double cY = k *
                IntStream.range(0, points.size())
                        .mapToDouble(i -> {
                            final Translation2d point = points.get(i);
                            final Translation2d nextPoint = (i + 1) >= points.size()
                                    ? points.get(0)
                                    : points.get(i + 1);

                            return (point.getY() + nextPoint.getY())
                                    * ((point.getX() * nextPoint.getY()) - (nextPoint.getX() * point.getY()));
                        })
                        .sum();

        return new Translation2d(cX, cY);
    }

    /**
     * Checks if a provided {@link Translation2d} lies within this {@link Boundary}; performs a point-inside-polygon
     * check.
     * <p>Algorithm derived from <a href="https://wrfranklin.org/Research/Short_Notes/pnpoly.html">here</a></p>
     * @param point the {@link Translation2d} to check the {@link Boundary} against
     * @return true if the {@link Boundary} contains this point, false if it does not
     */
    public boolean contains(final Translation2d point) {
        if (isRect) {
            // fast path for rectangles
            return rect.contains(point);
        }

        int i, j;
        boolean c = false;
        for (i = 0, j = points.size() - 1; i < points.size(); j = i++) {
            final Translation2d pi = points.get(i);
            final Translation2d pj = points.get(j);

            final double px = point.getX();
            final double py = point.getY();
            final double pix = pi.getX();
            final double piy = pi.getY();
            final double pjx = pj.getX();
            final double pjy = pj.getY();

            if ((piy > py) != (pjy > py) && (px < (pjx - pix) * (py - piy) / (pjy - piy) + pix)) {
                c = !c;
            }
        }
        return c;
    }
}
