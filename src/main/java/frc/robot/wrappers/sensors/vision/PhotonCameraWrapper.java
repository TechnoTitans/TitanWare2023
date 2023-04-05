package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.Optional;

public class PhotonCameraWrapper {
    private PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera apriltagCamera;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition;

    public PhotonCameraWrapper(PhotonCamera apriltagCamera) {
        this.apriltagCamera = apriltagCamera;
        this.apriltagCamera.setDriverMode(false);
        loadTags();
    }

    public void loadTags() {
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                robotOriginPosition = AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
            } else {
                robotOriginPosition = AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
            }
            fieldLayout.setOrigin(robotOriginPosition);

            photonPoseEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP,
                    apriltagCamera,
                    RobotMap.robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    public boolean robotOriginMatchesAlliance() {
        return (DriverStation.getAlliance() != DriverStation.Alliance.Blue || robotOriginPosition != AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide) &&
                (DriverStation.getAlliance() != DriverStation.Alliance.Red || robotOriginPosition != AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}