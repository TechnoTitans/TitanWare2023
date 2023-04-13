package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

@SuppressWarnings("unused")
public class PhotonApriltags extends SubsystemBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;
    private final PhotonRunnable photonEstimator;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition;

    public PhotonApriltags(PhotonCamera apriltagCamera, Swerve swerve, SwerveDrivePoseEstimator poseEstimator, Field2d field2d) {
        apriltagCamera.setDriverMode(false);
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.field2d = field2d;

        this.photonEstimator = new PhotonRunnable(apriltagCamera);

        try (Notifier photonNotifier = new Notifier(photonEstimator)) {
            photonNotifier.setName("PhotonRunnable");
            photonNotifier.startPeriodic(0.02);
        }
    }

    public void refreshAlliance() {
        boolean allianceChanged;
        switch (DriverStation.getAlliance()) {
            case Blue:
                allianceChanged = (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
                robotOriginPosition = AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
                robotOriginPosition = AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
                break;
            default:
                return;
        }

        if (allianceChanged) {
            Pose2d newPose = MathMethods.flipAlliancePose(poseEstimator.getEstimatedPosition());
            poseEstimator.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), newPose);
        }
    }

    @Override
    public void periodic() {
        EstimatedRobotPose estimatedRobotPose = photonEstimator.grabLatestEstimatedPose();
        if (estimatedRobotPose != null) {
            Pose2d estimatedPose2d = estimatedRobotPose.estimatedPose.toPose2d();
            if (robotOriginPosition != AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide) {
                estimatedPose2d = MathMethods.flipAlliancePose(estimatedPose2d);
            }
            poseEstimator.addVisionMeasurement(
                    estimatedPose2d,
                    estimatedRobotPose.timestampSeconds
            );
        }

        poseEstimator.update(
                swerve.getRotation2d(),
                swerve.getModulePositions());

        Pose2d dashboardPose = poseEstimator.getEstimatedPosition();
        if (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide) {
            dashboardPose = MathMethods.flipAlliancePose(dashboardPose);
        }
        field2d.setRobotPose(dashboardPose);
    }

    public boolean robotOriginMatchesAlliance() {
        return (DriverStation.getAlliance() != DriverStation.Alliance.Blue || robotOriginPosition != AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide) &&
                (DriverStation.getAlliance() != DriverStation.Alliance.Red || robotOriginPosition != AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    }
}