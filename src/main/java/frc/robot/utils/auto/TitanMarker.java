package frc.robot.utils.auto;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;

import java.util.Comparator;

public class TitanMarker extends EventMarker {
    private double timeSeconds = -1;
    private final Translation2d markerPosition;

    public TitanMarker(
            final double waypointRelativePos,
            final Command command,
            final double minimumTriggerDistance,
            final Translation2d markerPosition
    ) {
        super(waypointRelativePos, command, minimumTriggerDistance);
        this.markerPosition = markerPosition;
    }

    public TitanMarker(
            final double waypointRelativePos,
            final Command command,
            final double minimumTriggerDistance,
            final PathPlannerPath path
    ) {
        super(waypointRelativePos, command, minimumTriggerDistance);
        this.markerPosition = PathPlannerUtil.getMarkerPosition(path, this);
    }

    public double getTimeSeconds(final TitanTrajectory trajectory) {
        if (this.timeSeconds != -1) {
            return this.timeSeconds;
        } else {
            // TODO: this might work?
            return (this.timeSeconds = trajectory.getStates().stream()
                    .min(Comparator.comparingDouble(state -> state.positionMeters.getDistance(markerPosition)))
                    .orElseThrow()
                    .timeSeconds);
        }
    }

    public Translation2d getMarkerPosition() {
        return markerPosition;
    }

    public static TitanMarker fromEventMarker(final PathPlannerPath path, final EventMarker eventMarker) {
        return new TitanMarker(
                eventMarker.getWaypointRelativePos(),
                eventMarker.getCommand(),
                eventMarker.getMinimumTriggerDistance(),
                path
        );
    }

    public static TitanMarker transformMarkerForAlliance(
            final TitanMarker marker,
            final DriverStation.Alliance alliance
    ) {
        if (alliance == DriverStation.Alliance.Red) {
            final Translation2d translation = marker.getMarkerPosition();
            return new TitanMarker(
                    marker.getWaypointRelativePos(),
                    marker.getCommand(),
                    marker.getMinimumTriggerDistance(),
                    new Translation2d(
                            translation.getX(), FieldConstants.FIELD_WIDTH_Y_METERS - translation.getY()
                    )
            );
        } else {
            return marker;
        }
    }
}
