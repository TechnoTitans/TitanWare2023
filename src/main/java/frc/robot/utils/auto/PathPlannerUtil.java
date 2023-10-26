package frc.robot.utils.auto;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathSegment;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.util.Arrays;
import java.util.List;

public class PathPlannerUtil {
    public static List<File> getAllPathPlannerPaths() {
        final File[] pathPlannerPaths = new File(
                Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toString()
        ).listFiles();

        if (pathPlannerPaths == null || pathPlannerPaths.length == 0) {
            return List.of();
        }

        return Arrays.stream(pathPlannerPaths).toList();
    }

    public static String removeExtensionFromFilename(final String fileName) {
        return fileName.replaceFirst("[.][^.]+$", "");
    }

    public static List<String> getAllPathPlannerPathNames() {
        return getAllPathPlannerPaths().stream().map(file -> removeExtensionFromFilename(file.getName())).toList();
    }

    // TODO: get PathPlanner to make a way to do this without work on our end
    public static Translation2d getMarkerPosition(final PathPlannerPath path, final EventMarker marker) {
        final int pointIndex = (int) Math.round(marker.getWaypointRelativePos() / PathSegment.RESOLUTION);
        return path.getAllPathPoints().get(pointIndex).position;
    }
}
