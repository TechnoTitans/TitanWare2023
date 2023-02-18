package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    private final PhotonCamera photonCamera;
    private PhotonPipelineResult lastPipelineResult;

    public PhotonVision(PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
    }

    public Pose2d getRobotPoseRelativeToAprilTag() {
        lastPipelineResult = photonCamera.getLatestResult();
        if (lastPipelineResult.hasTargets()) {
            PhotonTrackedTarget bestTarget = lastPipelineResult.getBestTarget();
            // Single best target
            return new Pose2d(bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d(), bestTarget.getBestCameraToTarget().getRotation().toRotation2d());
        }
        return new Pose2d();
    }

    public boolean hasTargets() {
        lastPipelineResult = photonCamera.getLatestResult();
        return lastPipelineResult.hasTargets();
    }

    public int targetId() {
        return lastPipelineResult.getBestTarget().getFiducialId();
    }
}
