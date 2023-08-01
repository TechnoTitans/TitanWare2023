package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.pathfinding.bounds.NodeObstacle;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.*;

public class NodeField {
    protected static final String logKey = "NodeField";
    public static final int MAX_SEARCH_STEPS = 16;
    private final TranslationNode[][] nodes;

    private final double minCollisionDistanceX;
    private final double minCollisionDistanceY;
    private final double maxXBound;
    private final double maxYBound;
    private final double xDistance;
    private final double yDistance;

    private double create(final List<NodeObstacle> obstacles) {
        final double creationStart = Logger.getInstance().getRealTimestamp();

        // TODO: optimize everything/measure perf
        int iY = 0;
        for (double y = minCollisionDistanceY; y <= maxYBound; y += yDistance) {
            int iX = 0;
            for (double x = minCollisionDistanceX; x <= maxXBound; x += xDistance) {
                // TODO: optimize this maybe, or at least investigate its performance
                final double nodeCreateStart = Logger.getInstance().getRealTimestamp();

                final Translation2d point = new Translation2d(x, y);
                boolean isObstructed = false;
                for (final NodeObstacle obstacle : obstacles) {
                    if (obstacle.isObstructed(point)) {
                        isObstructed = true;
                        break;
                    }
                }

                final TranslationNode node = new TranslationNode(new Translation2d(x, y));
                nodes[iY][iX] = isObstructed ? null : node;

                Logger.getInstance().recordOutput(
                        logKey + "/PerNodeCreationTimeMS",
                        LogUtils.microsecondsToMilliseconds(Logger.getInstance().getRealTimestamp() - nodeCreateStart)
                );

                iX++;
            }

            iY++;
        }

        for (int i = 0; i < nodes.length; i++) {
            for (int j = 0; j < nodes[i].length; j++) {
                final double nodeNeighborComputeStart = Logger.getInstance().getRealTimestamp();

                final TranslationNode node = nodes[i][j];
                if (node == null) {
                    continue;
                }

                final List<Node<Translation2d>> neighbors = getNeighbors(nodes, i, j);
                node.addNeighbors(neighbors);

                Logger.getInstance().recordOutput(
                        logKey + "/PerNodeNeighborComputeTimeMS",
                        LogUtils.microsecondsToMilliseconds(
                                Logger.getInstance().getRealTimestamp() - nodeNeighborComputeStart
                        )
                );
            }
        }

        return Logger.getInstance().getRealTimestamp() - creationStart;
    }

    public NodeField(final int nodeDensity, final List<NodeObstacle> obstacles) {
        this.minCollisionDistanceX = Constants.Swerve.WHEEL_BASE * 0.5;
        this.minCollisionDistanceY = Constants.Swerve.TRACK_WIDTH * 0.5;

        this.maxXBound = FieldConstants.FIELD_LENGTH_X_METERS - minCollisionDistanceX;
        this.maxYBound = FieldConstants.FIELD_WIDTH_Y_METERS - minCollisionDistanceY;

        final double xLength = Math.abs(maxXBound - minCollisionDistanceX);
        final double yWidth = Math.abs(maxYBound - minCollisionDistanceY);

        final double meanDistance = 1 / Math.pow(nodeDensity, 0.5);
        final int xNodeCount = (int)(xLength / meanDistance);
        final int yNodeCount = (int)(yWidth / meanDistance);

        this.xDistance = xLength / xNodeCount;
        this.yDistance = yWidth / yNodeCount;

        this.nodes = new TranslationNode[yNodeCount + 1][xNodeCount + 1];

        final double creationTimeMS = LogUtils.microsecondsToMilliseconds(create(obstacles));
        Logger.getInstance().recordOutput(logKey + "/TotalCreationTimeMS", creationTimeMS);

//        Logger.getInstance().recordOutput(
//                "Nodes",
//                Arrays.stream(nodes)
//                        .flatMap(
//                                translationNodes ->
//                                        Arrays.stream(translationNodes)
//                                                .filter(Objects::nonNull)
//                                                .map(node -> new Pose2d(node.getValue(), new Rotation2d()))
//                        )
//                        .toArray(Pose2d[]::new)
//        );
    }

    public TranslationNode getNode(final double x, final double y) {
        final int ySize = nodes.length;
        final int iY;
        if (y <= minCollisionDistanceY) {
            iY = 0;
        } else if (y >= maxYBound) {
            iY = ySize - 1;
        } else {
            iY = Math.min((int)((y * ySize) / (maxYBound - minCollisionDistanceY)) - 1, ySize - 1);
        }

        final TranslationNode[] yNodes = nodes[iY];
        final int xSize = yNodes.length;
        final int iX;
        if (x <= minCollisionDistanceX) {
            iX = 0;
        } else if (x >= maxXBound) {
            iX = xSize - 1;
        } else {
            iX = Math.min((int)((x * xSize) / (maxXBound - minCollisionDistanceX)) - 1, xSize - 1);
        }

        final TranslationNode directNode = yNodes[iX];
        if (directNode != null) {
            return directNode;
        }

        int steps = 0;
        int oY = 1;
        int oX = 1;
        while (steps <= MAX_SEARCH_STEPS) {
            for (int k = iY - oY; k <= (iY + oY); k++) {
                for (int v = iX - oX; v <= (iX + oX); v++) {
                    if (k == iY && v == iX) {
                        continue;
                    }

                    if (k >= 0 && v >= 0 && k < nodes.length && v < nodes[k].length) {
                        final TranslationNode t = nodes[k][v];
                        if (t != null) {
                            return t;
                        }
                    }

                    steps++;
                }
            }

            oY++;
            oX++;
        }

        return null;
    }

    public TranslationNode getNode(final Translation2d translation2d) {
        return getNode(translation2d.getX(), translation2d.getY());
    }

    public Optional<List<Node<Translation2d>>> compute(final TranslationNode from, final TranslationNode to) {
        return AStar.compute(from, to);
    }

    public Optional<List<Node<Translation2d>>> compute(final Translation2d from, final Translation2d to) {
        final TranslationNode fromNode = getNode(from);
        final TranslationNode toNode = getNode(to);

        if (fromNode == null || toNode == null) {
            return Optional.empty();
        }

        return compute(fromNode, toNode);
    }

    public static <T> List<T> getNeighbors(final T[][] tArray, final int i, final int j) {
        final List<T> neighbors = new ArrayList<>(8);
        for (int k = i - 1; k <= (i + 1); k++) {
            for (int v = j - 1; v <= (j + 1); v++) {
                if (k == i && v == j) {
                    continue;
                }

                if (k >= 0 && v >= 0 && k < tArray.length && v < tArray[k].length) {
                    final T t = tArray[k][v];
                    if (t != null) {
                        neighbors.add(tArray[k][v]);
                    }
                }
            }
        }

        return neighbors;
    }
}
