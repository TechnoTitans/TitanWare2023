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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.autonomous.TrajectoryFollower;
import frc.robot.constants.Constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class TitanTrajectory extends PathPlannerTrajectory {
    private final TrajectoryFollower.FollowerContext followerContext;
    private final Map<EventMarker, SequentialCommandGroup> markerCommandMap;

    private TitanTrajectory(
            final List<State> states,
            final List<EventMarker> markers,
            final StopEvent startStopEvent,
            final StopEvent endStopEvent,
            final boolean fromGUI,
            final TrajectoryFollower.FollowerContext followerContext,
            final Map<EventMarker, SequentialCommandGroup> markerCommandMap
    ) {
        super(states, markers, startStopEvent, endStopEvent, fromGUI);
        this.followerContext = followerContext;
        this.markerCommandMap = markerCommandMap;
    }

    public TitanTrajectory(
            final List<State> states,
            final List<EventMarker> markers,
            final StopEvent startStopEvent,
            final StopEvent endStopEvent,
            final boolean fromGUI,
            final TrajectoryFollower.FollowerContext followerContext
    ) {
        this(
                states,
                markers,
                startStopEvent,
                endStopEvent,
                fromGUI,
                followerContext,
                markers.stream()
                        .collect(Collectors.toUnmodifiableMap(
                                eventMarker -> eventMarker,
                                eventMarker -> createCommandFromMarker(eventMarker, followerContext)
                        ))
        );
    }

    public TrajectoryFollower.FollowerContext getFollowerContext() {
        return followerContext;
    }

    public SequentialCommandGroup getCommandAtMarker(final EventMarker eventMarker) {
        if (followerContext == null) {
            throw new RuntimeException("Cannot get MarkerCommand for a TitanTrajectory without a FollowerContext!");
        }

        final SequentialCommandGroup commandGroup = markerCommandMap.get(eventMarker);
        if (commandGroup == null) {
            throw new RuntimeException("Cannot get invalid EventMarker from MarkerCommand!");
        }

        return commandGroup;
    }

    public static SequentialCommandGroup createCommandFromMarker(
            final EventMarker eventMarker,
            final TrajectoryFollower.FollowerContext followerContext
    ) {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        final String markerCommands = String.join("", eventMarker.names).trim();
        final String[] splitStringCommands = markerCommands.split(MarkerCommand.COMMAND_DELIMITER);

        for (final String stringCommand : splitStringCommands) {
            final Command command = MarkerCommand.get(stringCommand.toUpperCase(), followerContext);
            commandGroup.addCommands(command);
        }

        return commandGroup;
    }

    public static TitanTrajectory fromPathPlannerTrajectory(
            final PathPlannerTrajectory pathPlannerTrajectory,
            final TrajectoryFollower.FollowerContext followerContext
    ) {
        return new TitanTrajectory(
                pathPlannerTrajectory.getStates(),
                pathPlannerTrajectory.getMarkers(),
                pathPlannerTrajectory.getStartStopEvent(),
                pathPlannerTrajectory.getEndStopEvent(),
                pathPlannerTrajectory.fromGUI,
                followerContext
        );
    }

    public static EventMarker transformMarkerForAlliance(
            final EventMarker marker,
            final DriverStation.Alliance alliance
    ) {
        if (alliance == DriverStation.Alliance.Red) {
            final EventMarker transformedMarker = new EventMarker(marker.names, marker.waypointRelativePos);
            final Translation2d translation = marker.positionMeters;

            transformedMarker.timeSeconds = marker.timeSeconds;
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

            final Map<EventMarker, SequentialCommandGroup> transformedMarkerCommandMap = new HashMap<>();
            for (final EventMarker marker : markers) {
                final SequentialCommandGroup commandGroup = trajectory.markerCommandMap.get(marker);
                final EventMarker transformedMarker = transformMarkerForAlliance(marker, alliance);

                transformedMarkerCommandMap.put(transformedMarker, commandGroup);
                transformedMarkers.add(transformedMarker);
            }

            return new TitanTrajectory(
                    transformedStates,
                    transformedMarkers,
                    trajectory.getStartStopEvent(),
                    trajectory.getEndStopEvent(),
                    trajectory.fromGUI,
                    trajectory.followerContext,
                    transformedMarkerCommandMap
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

        public TitanTrajectory build(final TrajectoryFollower.FollowerContext followerContext) {
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

            return TitanTrajectory.fromPathPlannerTrajectory(
                    PathPlanner.generatePath(constraints, regenerated),
                    followerContext
            );
        }
    }
}
