package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import java.util.List;
import java.util.Map;

public class PhotonApriltags extends SubsystemBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;
    private final PhotonRunnable photonEstimator;
    private final Notifier photonNotifier;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition;

    public PhotonApriltags(
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final Field2d field2d,
            final Map<PhotonCamera, Transform3d> apriltagCameras
    ) {
        apriltagCameras.keySet().forEach(camera -> camera.setDriverMode(false));

        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.field2d = field2d;

        this.photonEstimator = new PhotonRunnable(apriltagCameras);

        refreshAlliance();

        photonNotifier = new Notifier(photonEstimator);
        photonNotifier.setName("PhotonRunnable");
        photonNotifier.startPeriodic(0.02);
    }

    public void refreshAlliance() {
        final boolean allianceChanged;
        switch (DriverStation.getAlliance()) {
            case Blue -> {
                allianceChanged = (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
                robotOriginPosition = AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
            }
            case Red -> {
                allianceChanged = (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
                robotOriginPosition = AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
            }
            default -> {
                return;
            }
        }

        if (allianceChanged) {
            final Pose2d newPose = PoseUtils.flipPose(poseEstimator.getEstimatedPosition());
            poseEstimator.resetPosition(swerve.getYawRotation2d(), swerve.getModulePositions(), newPose);
        }
    }

    @Override
    public void periodic() {
        final List<EstimatedRobotPose> estimatedRobotPoses = photonEstimator.grabLatestEstimatedPoseRight();

        for (final EstimatedRobotPose estimatedRobotPose : estimatedRobotPoses) {
            Pose2d estimatedPose2d = estimatedRobotPose.estimatedPose.toPose2d();
            if (robotOriginPosition != AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide) {
                estimatedPose2d = PoseUtils.flipPose(estimatedPose2d);
            }
            poseEstimator.addVisionMeasurement(
                    estimatedPose2d,
                    estimatedRobotPose.timestampSeconds
            );
        }

        poseEstimator.update(
                swerve.getYawRotation2d(),
                swerve.getModulePositions()
        );

        final Pose2d dashboardPose = poseEstimator.getEstimatedPosition();

        field2d.setRobotPose(dashboardPose);
        Logger.getInstance().recordOutput("Odometry/Robot", dashboardPose);
        Logger.getInstance().recordOutput("PoseEstimatorRotation_Rad", dashboardPose.getRotation().getRadians());
        Logger.getInstance().recordOutput("SwerveRotation_Rad", swerve.getYawRotation2d().getRadians());
    }

    public Notifier getPhotonNotifier() {
        return photonNotifier;
    }
}