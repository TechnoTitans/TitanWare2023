package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.util.Optional;

import frc.robot.RobotMap;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
    private PhotonPoseEstimator photonPoseEstimator;

    public PhotonCameraWrapper(PhotonCamera apriltagCamera) {
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            fieldLayout.setOrigin(
                    (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) ?
                            AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide :
                            AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide
            );
            photonPoseEstimator = new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP, apriltagCamera, RobotMap.robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}