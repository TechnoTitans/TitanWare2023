package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class PhotonRunnable implements Runnable {
    public static final boolean IGNORE_DURING_AUTO = true;

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera apriltagCamera;
    private final AtomicReference<EstimatedRobotPose> atomicLastEstimatedRobotPose = new AtomicReference<>();
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<>();

    private final AtomicLong lastVisionTimestamp = new AtomicLong(RobotController.getFPGATime());

    public PhotonRunnable(PhotonCamera apriltagCamera) {
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
        if (photonPoseEstimator != null
                && apriltagCamera != null
                && (!IGNORE_DURING_AUTO || !RobotState.isAutonomous())
        ) {
            final PhotonPipelineResult photonResults = apriltagCamera.getLatestResult();
            if (photonResults.hasTargets()
                    && (photonResults.targets.size() > 1
                        || photonResults.targets.get(0).getPoseAmbiguity() < Constants.Vision.singleTagMaxAmbiguity
            )) {
                photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    final Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                    final Pose2d flattenedEstimatedPose = estimatedPose.toPose2d();

                    final double lastTimestamp = Double.longBitsToDouble(
                            lastVisionTimestamp.getAndSet(Double.doubleToLongBits(estimatedRobotPose.timestampSeconds))
                    );

                    if (estimatedPose.getX() < 0.0 || estimatedPose.getX() > Constants.Grid.FIELD_LENGTH_METERS
                            || estimatedPose.getY() < 0.0 || estimatedPose.getY() > Constants.Grid.FIELD_WIDTH_METERS) {
                        // ignore estimated pose if it's out of the field
                        return;
                    }

                    final EstimatedRobotPose lastEstimatedRobotPose = atomicLastEstimatedRobotPose.get();
                    if (lastEstimatedRobotPose == null) {
                        atomicLastEstimatedRobotPose.compareAndSet(null, estimatedRobotPose);
                        return;
                    }

                    final double distanceToLastEstimatedPose = PhotonUtils.getDistanceToPose(
                            lastEstimatedRobotPose.estimatedPose.toPose2d(), flattenedEstimatedPose
                    );

                    final double avgVelocityOverLastMeasurement =
                            distanceToLastEstimatedPose /
                                    (estimatedRobotPose.timestampSeconds - lastTimestamp);

                    if (Math.abs(avgVelocityOverLastMeasurement) > Constants.Swerve.ROBOT_MAX_SPEED) {
                        // ignore estimated pose if the average velocity is greater than the max speed
                        return;
                    }

                    atomicEstimatedRobotPose.set(estimatedRobotPose);
                    atomicLastEstimatedRobotPose.set(estimatedRobotPose);
                });
            }
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }
}
