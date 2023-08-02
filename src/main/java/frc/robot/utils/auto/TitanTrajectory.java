package frc.robot.utils.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class TitanTrajectory extends PathPlannerTrajectory {
    public TitanTrajectory(
            final List<State> states,
            final List<EventMarker> markers,
            final StopEvent startStopEvent,
            final StopEvent endStopEvent,
            final boolean fromGUI
    ) {
        super(states, markers, startStopEvent, endStopEvent, fromGUI);
    }

    public static TitanTrajectory fromPathPlannerTrajectory(final PathPlannerTrajectory pathPlannerTrajectory) {
        return new TitanTrajectory(
                pathPlannerTrajectory.getStates(),
                pathPlannerTrajectory.getMarkers(),
                pathPlannerTrajectory.getStartStopEvent(),
                pathPlannerTrajectory.getEndStopEvent(),
                pathPlannerTrajectory.fromGUI
        );
    }

    public static EventMarker transformMarkerForAlliance(
            final EventMarker marker,
            final DriverStation.Alliance alliance
    ) {
        if (alliance == DriverStation.Alliance.Red) {
            final EventMarker transformedMarker = new EventMarker(marker.names, marker.waypointRelativePos);
            final Translation2d translation = marker.positionMeters;

            transformedMarker.positionMeters = new Translation2d(
                    translation.getX(), FieldConstants.FIELD_WIDTH_Y_METERS - translation.getY()
            );

            return transformedMarker;
        } else {
            return marker;
        }
    }

    public static TitanTrajectory transformTrajectoryForAlliance(
            final TitanTrajectory trajectory,
            final DriverStation.Alliance alliance
    ) {
        if (alliance == DriverStation.Alliance.Red) {
            final List<State> states = trajectory.getStates();
            final List<EventMarker> markers = trajectory.getMarkers();
            final List<State> transformedStates = new ArrayList<>(states.size());
            final List<EventMarker> transformedMarkers = new ArrayList<>(markers.size());

            for (final State state : states) {
                final PathPlannerState pathPlannerState = (PathPlannerState) state;
                transformedStates.add(transformStateForAlliance(pathPlannerState, alliance));
            }

            for (final EventMarker marker : markers) {
                transformedMarkers.add(transformMarkerForAlliance(marker, alliance));
            }

            return new TitanTrajectory(
                    transformedStates,
                    transformedMarkers,
                    trajectory.getStartStopEvent(),
                    trajectory.getEndStopEvent(),
                    trajectory.fromGUI
            );
        } else {
            return trajectory;
        }
    }

    public static class Constraints extends PathConstraints {
        public final double maxAngularVelocity;
        public final double maxAngularAcceleration;

        public Constraints(
                final double maxLinearVelocity,
                final double maxLinearAcceleration,
                final double maxAngularVelocity,
                final double maxAngularAcceleration
        ) {
            super(maxLinearVelocity, maxLinearAcceleration);
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxAngularAcceleration = maxAngularAcceleration;
        }

        public static Constraints getDefault() {
            return new Constraints(
                    Constants.Swerve.TRAJECTORY_MAX_SPEED,
                    Constants.Swerve.TRAJECTORY_MAX_ACCELERATION,
                    Constants.Swerve.TRAJECTORY_MAX_ANGULAR_SPEED,
                    Constants.Swerve.TRAJECTORY_MAX_ANGULAR_ACCELERATION
            );
        }
    }

    public static class Builder {
        public static final Rotation2d TEMP_HEADING_HANDLE = Rotation2d.fromDegrees(180);
        private final List<PathPoint> pathPoints;

        private double endVelocityOverride = -1;
        private Constraints constraints = Constraints.getDefault();

        public Builder(final List<PathPoint> pathPoints) {
            this.pathPoints = pathPoints;
        }

        public Builder() {
            this(new ArrayList<>());
        }

        public Builder add(final PathPoint pathPoint) {
            if (pathPoints.isEmpty()) {
                pathPoints.add(pathPoint);
            } else {
                final PathPoint lastPoint = pathPoints.get(pathPoints.size() - 1);

                final Translation2d transformFromLast = pathPoint.position.minus(lastPoint.position);
                final double approxDistance = transformFromLast.getNorm();
                final double approxTime = approxDistance / constraints.maxVelocity;

                final double approxAngularRotation = constraints.maxAngularVelocity * approxTime;

                final Rotation2d deltaHolonomic = pathPoint.holonomicRotation.minus(lastPoint.holonomicRotation);
                final Rotation2d clampedHolonomic = Rotation2d.fromRadians(
                        MathUtil.clamp(deltaHolonomic.getRadians(), -approxAngularRotation, approxAngularRotation)
                );

                pathPoints.add(new PathPoint(
                        pathPoint.position,
                        pathPoint.heading,
                        lastPoint.holonomicRotation.plus(clampedHolonomic),
                        pathPoint.velocityOverride
                ));
            }

            return this;
        }

        public Builder add(final Pose2d currentPose, final ChassisSpeeds robotRelativeSpeeds) {
            if (!pathPoints.isEmpty()) {
                throw new RuntimeException("attempted to add initial PathPoint when points was not empty!");
            }

            return add(PathPoint.fromCurrentHolonomicState(currentPose, robotRelativeSpeeds));
        }

        public Builder add(
                final Translation2d position,
                final Rotation2d heading,
                final Rotation2d holonomicRotation,
                final double velocityOverride
        ) {
            return add(new PathPoint(position, heading, holonomicRotation, velocityOverride));
        }

        public Builder add(
                final Translation2d position,
                final Rotation2d holonomicRotation
        ) {
            return add(new PathPoint(position, TEMP_HEADING_HANDLE, holonomicRotation));
        }

        public Builder add(final Pose2d positionHolonomic) {
            add(positionHolonomic.getTranslation(), positionHolonomic.getRotation());
            return this;
        }

        public Builder withEndVelocityOverride(final double velocityOverride) {
            if (velocityOverride < 0) {
                throw new IllegalArgumentException("velocityOverride cannot be < 0!");
            }

            this.endVelocityOverride = velocityOverride;
            return this;
        }

        public Builder withConstraints(final Constraints constraints) {
            this.constraints = constraints;
            return this;
        }

        public static Rotation2d getHeading(final PathPoint point, final PathPoint last, final PathPoint next) {
            final Rotation2d heading;
            if (last == null && next != null) {
                final Translation2d diff = next.position.minus(point.position);
                heading = new Rotation2d(diff.getX(), diff.getY());
            } else if (last != null && next == null) {
                final Translation2d diff = point.position.minus(last.position);
                heading = new Rotation2d(diff.getX(), diff.getY());
            } else if (next != null) {
                final Translation2d diff = next.position.minus(point.position);
                heading = new Rotation2d(diff.getX(), diff.getY());
            } else {
                heading = Rotation2d.fromRadians(TEMP_HEADING_HANDLE.getRadians());
            }

            return heading;
        }

        public TitanTrajectory build() {
            final List<PathPoint> regenerated = IntStream.range(0, pathPoints.size())
                    .mapToObj(i -> {
                        final PathPoint point = pathPoints.get(i);
                        final PathPoint lastPoint = (i > 0) ? pathPoints.get(i - 1) : null;
                        final PathPoint nextPoint = (i < (pathPoints.size() - 1)) ? pathPoints.get(i + 1) : null;

                        final Rotation2d heading = point.heading == TEMP_HEADING_HANDLE
                                ? getHeading(point, lastPoint, nextPoint)
                                : point.heading;

                        final double velocityOverride = nextPoint == null && endVelocityOverride != -1
                                ? endVelocityOverride
                                : point.velocityOverride;

                        return new PathPoint(
                                point.position,
                                heading,
                                point.holonomicRotation,
                                velocityOverride
                        );
                    })
                    .toList();

            return TitanTrajectory.fromPathPlannerTrajectory(PathPlanner.generatePath(constraints, regenerated));
        }
    }
}
