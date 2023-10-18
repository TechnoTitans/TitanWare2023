package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.constants.Constants;
import frc.robot.utils.vision.TitanCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class PhotonRunnable implements Runnable {
    public final String logKey;

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedPose = new AtomicReference<>();
    /**
     * A stable {@link AtomicReference} to a {@link EstimatedRobotPose}, this does NOT get set to null after we get
     * the estimated pose, thus, it is stable and represents the last estimated pose.
     * <p>
     * Do <b>NOT</b> use this {@link EstimatedRobotPose} to feed into vision adjustments.
     */
    private final AtomicReference<EstimatedRobotPose> atomicLastStableEstimatedPose = new AtomicReference<>();

    public PhotonRunnable(final TitanCamera titanCamera, final AprilTagFieldLayout aprilTagFieldLayout) {
        this.photonCamera = titanCamera.getPhotonCamera();
        this.logKey = String.format("%s_PoseEstimator", photonCamera.getName());
        this.poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP,
                photonCamera,
                titanCamera.getRobotRelativeToCameraTransform()
        );

        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    private void updatePoseEstimator(final PhotonPipelineResult photonPipelineResult) {
//        Logger.getInstance().recordOutput(
//                logKey + "/PipelineResultTargets",
//                photonPipelineResult.targets.stream().mapToDouble(PhotonTrackedTarget::getFiducialId).toArray()
//        );

//        Logger.getInstance().recordOutput(logKey + "/PrimaryStrategy", poseEstimator.getPrimaryStrategy().toString());

        final Optional<EstimatedRobotPose> optionalEstimatedRobotPose = poseEstimator.update(photonPipelineResult);

//        Logger.getInstance().recordOutput(logKey + "/OptionalIsPresent", optionalEstimatedRobotPose.isPresent());
        optionalEstimatedRobotPose.ifPresent(estimatedRobotPose -> {
            atomicEstimatedPose.set(estimatedRobotPose);
            atomicLastStableEstimatedPose.set(estimatedRobotPose);
        });
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    @Override
    public void run() {
        final PhotonPipelineResult photonResult = photonCamera.getLatestResult();
        if (!photonResult.hasTargets()) {
            atomicLastStableEstimatedPose.set(null);
            return;
        }

        final List<PhotonTrackedTarget> targets = photonResult.targets;
        final int nTargets = targets.size();

        if (nTargets == 1) {
            // single-tag
            final PhotonTrackedTarget firstTarget = targets.get(0);
            if (firstTarget.getPoseAmbiguity() <= Constants.Vision.SINGLE_TAG_MAX_AMBIGUITY) {
                updatePoseEstimator(photonResult);
            }
        } else {
            // multi-tag
            final List<PhotonTrackedTarget> filteredTargets = photonResult.getTargets()
                    .stream()
                    .filter(
                            photonTrackedTarget ->
                                    photonTrackedTarget.getPoseAmbiguity() <= Constants.Vision.MULTI_TAG_MAX_AMBIGUITY
                    )
                    .toList();

            final PhotonPipelineResult filteredPhotonPipelineResult = new PhotonPipelineResult(
                    photonResult.getLatencyMillis(), filteredTargets
            );

            filteredPhotonPipelineResult.setTimestampSeconds(photonResult.getTimestampSeconds());
            updatePoseEstimator(filteredPhotonPipelineResult);
        }
    }

    public EstimatedRobotPose getLatestEstimatedPose() {
        return atomicEstimatedPose.getAndSet(null);
    }

    public EstimatedRobotPose getStableLastEstimatedPose() {
        return atomicLastStableEstimatedPose.get();
    }
}
