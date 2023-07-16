package frc.robot.utils.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;

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
                    translation.getX(), Constants.Field.FIELD_WIDTH_Y_METERS - translation.getY()
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
}
