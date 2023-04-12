package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.Optional;

@SuppressWarnings("unused")
public class PhotonCameraWrapper extends SubsystemBase {
    private final PhotonCamera apriltagCamera;
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition;

    private double previousPipelineTimestamp = 0;

    public PhotonCameraWrapper(PhotonCamera apriltagCamera, Swerve swerve, SwerveDrivePoseEstimator poseEstimator, Field2d field2d) {
        this.apriltagCamera = apriltagCamera;
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.field2d = field2d;

        this.apriltagCamera.setDriverMode(false);

        try {
            fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

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

    @Override
    public void periodic() {
        var pipelineResult = apriltagCamera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        if (!DriverStation.isAutonomous() && resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
            previousPipelineTimestamp = resultTimestamp;
            var target = pipelineResult.getBestTarget();
            var fiducialId = target.getFiducialId();
            Optional<Pose3d> tagPose = fieldLayout == null ? Optional.empty() : fieldLayout.getTagPose(fiducialId);
            if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
                var targetPose = tagPose.get();
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                var visionMeasurement = camPose.transformBy(RobotMap.robotToCam);
                poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }

        poseEstimator.update(
                swerve.getRotation2d(),
                swerve.getModulePositions());

        field2d.getObject("robot").setPose(poseEstimator.getEstimatedPosition());
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