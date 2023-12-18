package frc.robot.utils.auto;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.autonomous.TrajectoryFollower;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class TitanTrajectory extends PathPlannerTrajectory {
    private final List<State> states;
    private final List<TitanMarker> eventMarkers;
    private final PathPlannerPath pathPlannerPath;
    private final TrajectoryFollower.FollowerContext followerContext;

    public TitanTrajectory(
            final List<State> states,
            final List<TitanMarker> eventMarkers,
            final PathPlannerPath pathPlannerPath,
            final TrajectoryFollower.FollowerContext followerContext
    ) {
        super(pathPlannerPath, followerContext.getInitialChassisSpeeds());
        this.states = states;
        this.eventMarkers = eventMarkers;
        this.pathPlannerPath = pathPlannerPath;
        this.followerContext = followerContext;
    }

    public TitanTrajectory(
            final PathPlannerPath pathPlannerPath,
            final TrajectoryFollower.FollowerContext followerContext
    ) {
        super(pathPlannerPath, followerContext.getInitialChassisSpeeds());
        this.states = super.getStates();
        this.eventMarkers = pathPlannerPath.getEventMarkers().stream()
                .map(eventMarker -> TitanMarker.fromEventMarker(pathPlannerPath, eventMarker))
                .toList();
        this.pathPlannerPath = pathPlannerPath;
        this.followerContext = followerContext;
    }

    @Override
    public List<State> getStates() {
        return states;
    }

    public List<TitanMarker> getEventMarkers() {
        return eventMarkers;
    }

    public TrajectoryFollower.FollowerContext getFollowerContext() {
        return followerContext;
    }

    public static TitanTrajectory fromPathPlannerPath(
            final PathPlannerPath pathPlannerPath,
            final TrajectoryFollower.FollowerContext followerContext
    ) {
        return new TitanTrajectory(
                pathPlannerPath,
                followerContext
        );
    }

    public static State transformStateForAlliance(final State state, final DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            // Create a new state so that we don't overwrite the original
            final State transformedState = new State();

            final Translation2d transformedTranslation = new Translation2d(
                    state.positionMeters.getX(),
                    FieldConstants.FIELD_WIDTH_Y_METERS - state.positionMeters.getY()
            );

            final Rotation2d transformedHeading = state.heading.times(-1);
            final Rotation2d transformedHolonomicRotation = state.targetHolonomicRotation.times(-1);

            transformedState.timeSeconds = state.timeSeconds;
            transformedState.velocityMps = state.velocityMps;
            transformedState.accelerationMpsSq = state.accelerationMpsSq;
            transformedState.headingAngularVelocityRps = -state.headingAngularVelocityRps;
            transformedState.positionMeters = transformedTranslation;
            transformedState.heading = transformedHeading;
            transformedState.targetHolonomicRotation = transformedHolonomicRotation;
            transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

            return transformedState;
        } else {
            return state;
        }
    }

    public static TitanTrajectory transformTrajectoryForAlliance(
            final TitanTrajectory trajectory,
            final DriverStation.Alliance alliance
    ) {
        if (alliance == DriverStation.Alliance.Red) {
            final List<State> states = trajectory.getStates();
            final List<TitanMarker> markers = trajectory.getEventMarkers();

            final List<State> transformedStates = new ArrayList<>(states.size());
            final List<TitanMarker> transformedMarkers = new ArrayList<>(markers.size());

            for (final State state : states) {
                transformedStates.add(transformStateForAlliance(state, alliance));
            }

            for (final TitanMarker marker : markers) {
                final TitanMarker transformedMarker = TitanMarker.transformMarkerForAlliance(marker, alliance);
                transformedMarkers.add(transformedMarker);
            }

            return new TitanTrajectory(
                    transformedStates,
                    transformedMarkers,
                    trajectory.pathPlannerPath,
                    trajectory.followerContext
            );
        } else {
            return trajectory;
        }
    }

    public static class Constraints extends PathConstraints {
        public Constraints(
                final double maxLinearVelocity,
                final double maxLinearAcceleration,
                final double maxAngularVelocity,
                final double maxAngularAcceleration
        ) {
            super(maxLinearVelocity, maxLinearAcceleration, maxAngularVelocity, maxAngularAcceleration);
        }

        public Constraints(
                final double maxLinearVelocity,
                final double maxLinearAcceleration
        ) {
            this(
                    maxLinearVelocity,
                    maxLinearAcceleration,
                    Constants.Swerve.TRAJECTORY_MAX_ANGULAR_SPEED,
                    Constants.Swerve.TRAJECTORY_MAX_ANGULAR_ACCELERATION
            );
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
                final double approxTime = approxDistance / constraints.getMaxVelocityMps();

                final double approxAngularRotation = constraints.getMaxAngularVelocityRps() * approxTime;

                final Rotation2d deltaHolonomic = pathPoint.rotationTarget.getTarget().minus(lastPoint.rotationTarget.getTarget());
                final Rotation2d clampedHolonomic = Rotation2d.fromRadians(
                        MathUtil.clamp(deltaHolonomic.getRadians(), -approxAngularRotation, approxAngularRotation)
                );

                pathPoints.add(new PathPoint(
                        pathPoint.position,
                        new RotationTarget(
                                pathPoint.rotationTarget.getPosition(),
                                lastPoint.rotationTarget.getTarget().plus(clampedHolonomic),
                                pathPoint.rotationTarget.shouldRotateFast()
                        ),
                        constraints
                ));
            }

            return this;
        }

//        public Builder add(final Pose2d currentPose, final ChassisSpeeds robotRelativeSpeeds) {
//            if (!pathPoints.isEmpty()) {
//                throw new RuntimeException("attempted to add initial PathPoint when points was not empty!");
//            }
//
//            return add(new PathPoint.fromCurrentHolonomicState(currentPose, robotRelativeSpeeds));
//        }

        public Builder add(
                final Translation2d position,
                final Rotation2d holonomicRotation
        ) {
            return add(new PathPoint(position, holonomicRotation));
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

        public TitanTrajectory build(final TrajectoryFollower.FollowerContext followerContext) {
            final List<PathPoint> regenerated = IntStream.range(0, pathPoints.size())
                    .mapToObj(i -> {
                        final PathPoint point = pathPoints.get(i);
                        final PathPoint lastPoint = (i > 0) ? pathPoints.get(i - 1) : null;
                        final PathPoint nextPoint = (i < (pathPoints.size() - 1)) ? pathPoints.get(i + 1) : null;

//                        final Rotation2d heading = point.heading == TEMP_HEADING_HANDLE
//                                ? getHeading(point, lastPoint, nextPoint)
//                                : point.heading;

//                        final double velocityOverride = nextPoint == null && endVelocityOverride != -1
//                                ? endVelocityOverride
//                                : point.velocityOverride;

                        return new PathPoint(
                                point.position,
//                                heading,
                                point.holonomicRotation
//                                velocityOverride
                        );
                    })
                    .toList();

            return TitanTrajectory.fromPathPlannerPath(
                    PathPlannerPath.fromPathPoints(
                            regenerated,
                            constraints,
                            new GoalEndState(0, Rotation2d.fromDegrees(0))
                    ),
                    followerContext
            );
        }
    }
}
