package frc.robot.utils.auto;

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
}
