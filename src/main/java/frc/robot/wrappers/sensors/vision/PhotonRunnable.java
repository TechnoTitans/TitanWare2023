package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

public class PhotonRunnable implements Runnable {
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera apriltagCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<>();

    public PhotonRunnable(final PhotonCamera apriltagCamera) {
        this.apriltagCamera = apriltagCamera;

        PhotonPoseEstimator photonPoseEstimator = null;
        try {
            AprilTagFieldLayout layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            photonPoseEstimator = new PhotonPoseEstimator(
                    layout,
                    PoseStrategy.MULTI_TAG_PNP,
                    apriltagCamera,
                    Constants.Vision.robotToCam);

            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        }
        this.photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        if (photonPoseEstimator != null && apriltagCamera != null && !RobotState.isAutonomous()) {
            final PhotonPipelineResult photonResults = apriltagCamera.getLatestResult();
            if (photonResults.hasTargets() &&
                    (photonResults.targets.size() > 1 ||
                            photonResults.targets.get(0).getPoseAmbiguity() < Constants.Vision.singleTagMaxAmbiguity)
            ) {
                photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    final Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= Constants.Field.FIELD_LENGTH_X_METERS
                            && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= Constants.Field.FIELD_WIDTH_Y_METERS) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

}
