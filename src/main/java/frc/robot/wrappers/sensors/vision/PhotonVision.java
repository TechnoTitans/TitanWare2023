package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    private final PhotonCamera photonCamera;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private PhotonPipelineResult lastPipelineResult;

    public PhotonVision(PhotonCamera photonCamera, SwerveDriveOdometry odometry, SwerveDrivePoseEstimator poseEstimator) {
        this.photonCamera = photonCamera;
        this.odometry = odometry;
        this.poseEstimator = poseEstimator;
    }

    public Transform3d getRobotTransformationRelativeToAprilTag() {
        lastPipelineResult = photonCamera.getLatestResult();
        if (lastPipelineResult.hasTargets()) {
            PhotonTrackedTarget bestTarget = lastPipelineResult.getBestTarget();
            // Single best target
            return bestTarget.getBestCameraToTarget();
        }
        return new Transform3d();
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
}
