package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class PhotonRunnable implements Runnable {
    private final Map<PhotonCamera, PhotonPoseEstimator> photonPoseCameraEstimatorMap;
    private final AtomicReference<List<EstimatedRobotPose>> photonEstimatedPoses = new AtomicReference<>(new ArrayList<>());

    public PhotonRunnable(final Map<PhotonCamera, Transform3d> apriltagCameras) {
        Map<PhotonCamera, PhotonPoseEstimator> tempPhotonPoseCameraEstimatorMap;
        try {
            final AprilTagFieldLayout layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            tempPhotonPoseCameraEstimatorMap = apriltagCameras.entrySet().stream().collect(
                    Collectors.toUnmodifiableMap(
                            Map.Entry::getKey,
                            cameraEntry -> {
                                final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
                                        layout, PoseStrategy.MULTI_TAG_PNP, cameraEntry.getKey(), cameraEntry.getValue()
                                );
                                photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                                return photonPoseEstimator;
                            }
                    )
            );
        } catch (final IOException e) {
            tempPhotonPoseCameraEstimatorMap = Map.of();
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        }
        this.photonPoseCameraEstimatorMap = tempPhotonPoseCameraEstimatorMap;
    }

    @Override
    public void run() {
        for (final Map.Entry<PhotonCamera, PhotonPoseEstimator> cameraEntry : photonPoseCameraEstimatorMap.entrySet()) {
            final PhotonPipelineResult photonResults = cameraEntry.getKey().getLatestResult();

            if (photonResults.hasTargets()
                    && (photonResults.targets.size() > 1
                    || photonResults.targets.get(0).getPoseAmbiguity() < Constants.Vision.singleTagMaxAmbiguity)
            ) {
                cameraEntry.getValue().update(photonResults).ifPresent(estimatedRobotPose -> {
                    final Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= Constants.Field.FIELD_LENGTH_X_METERS
                            && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= Constants.Field.FIELD_WIDTH_Y_METERS) {

                        photonEstimatedPoses.get().add(estimatedRobotPose);
                    }
                });
            }
        }
    }

    public List<EstimatedRobotPose> grabLatestEstimatedPoseRight() {
        return photonEstimatedPoses.getAndSet(new ArrayList<>());
    }
}
