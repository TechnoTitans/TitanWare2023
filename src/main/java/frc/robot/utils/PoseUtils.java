package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class PoseUtils {
    public static boolean poseWithinArea(Pose2d currentPose, Translation2d point1, Translation2d point2) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            point1 = new Translation2d(point1.getX(), Constants.Grid.FIELD_WIDTH_METERS - point1.getY());
            point2 = new Translation2d(point2.getX(), Constants.Grid.FIELD_WIDTH_METERS - point2.getY());
        }
        return currentPose.getTranslation().getX() > point1.getX() && currentPose.getTranslation().getX() < point2.getX() &&
                currentPose.getTranslation().getY() > point1.getY() && currentPose.getTranslation().getY() < point2.getY();
    }

    public static Pose2d flipAlliancePose(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(Constants.Grid.FLIPPING_POSE);
    }

    public static Pose2d transformPose(Pose2d originalPose) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return new Pose2d(
                    originalPose.getX(),
                    Constants.Grid.FIELD_WIDTH_METERS - originalPose.getY(),
                    originalPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        } else {
            return originalPose;
        }
    }
}
