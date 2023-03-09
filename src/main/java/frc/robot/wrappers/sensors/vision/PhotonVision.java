package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    private final PhotonCamera photonCamera;

    public PhotonVision(PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        photonCamera.setDriverMode(true);
    }

    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

    public Pose2d getRobotPoseRelativeToAprilTag(PhotonPipelineResult pipelineResult) {
        if (hasTargets(pipelineResult)) {
            PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
            // Single best target
            return new Pose2d(bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d(), bestTarget.getBestCameraToTarget().getRotation().toRotation2d());
        }
        return new Pose2d();
    }

    public boolean hasTargets(PhotonPipelineResult pipelineResult) {
        return pipelineResult.hasTargets();
    }

    public int targetId(PhotonPipelineResult pipelineResult) {
        return pipelineResult.hasTargets() ? pipelineResult.getBestTarget().getFiducialId() : -1;
    }
}
