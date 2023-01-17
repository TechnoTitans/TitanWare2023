package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

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

    public Transform3d getRobotPoseRelativeToAprilTag() {
        lastPipelineResult = photonCamera.getLatestResult();
        if (lastPipelineResult.hasTargets()) {
            PhotonTrackedTarget bestTarget = lastPipelineResult.getBestTarget();
            // Single best target
            return bestTarget.getBestCameraToTarget();
        }
        return null;
    }

    public void testing() {
        List<Transform3d> targets = new ArrayList<>();
        lastPipelineResult = photonCamera.getLatestResult();
        lastPipelineResult.getTargets().forEach(target -> {
            // Making sure targets are accurate
            if (target.getPoseAmbiguity() <= 0.15) { //TODO: TUNE AMBIGUITY VALUE
                targets.add(target.getBestCameraToTarget());
            }
        });
        if (targets.size() > 0) {
            double x = 0, y = 0, z = 0, theta = 0;

            for (Transform3d target : targets) {
                x += target.getX();
                y += target.getY();
                z += target.getZ();
                theta += target.getRotation().getAngle();
            }
            x /= targets.size();
            y /= targets.size();
            z /= targets.size();
            theta /= targets.size();

//            Pose3d averageTarget = new Pose3d(x, y, z, Rotation3d.fromDegrees(theta)); im to tied ill play with this later
        }


    }

}
