package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

@SuppressWarnings("unused")
public class PhotonApriltags extends SubsystemBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;
    private final PhotonRunnable photonEstimator;
    private final Notifier photonNotifier;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition;

    public PhotonApriltags(
            final PhotonCamera apriltagCamera,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final Field2d field2d
    ) {
        apriltagCamera.setDriverMode(false);
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.field2d = field2d;

        this.photonEstimator = new PhotonRunnable(apriltagCamera);

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
            final Pose2d newPose = PoseUtils.flipAlliancePose(poseEstimator.getEstimatedPosition());
            poseEstimator.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), newPose);
        }
    }

    @Override
    public void periodic() {
        final EstimatedRobotPose estimatedRobotPose = photonEstimator.grabLatestEstimatedPose();
        if (estimatedRobotPose != null) {
            Pose2d estimatedPose2d = estimatedRobotPose.estimatedPose.toPose2d();
            if (robotOriginPosition != AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide) {
                estimatedPose2d = PoseUtils.flipAlliancePose(estimatedPose2d);
            }
            poseEstimator.addVisionMeasurement(
                    estimatedPose2d,
                    estimatedRobotPose.timestampSeconds
            );
        }

        poseEstimator.update(
                swerve.getRotation2d(),
                swerve.getModulePositions()
        );

        Pose2d dashboardPose = poseEstimator.getEstimatedPosition();
        //TODO: this thing is suspicious!
//        if (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide) {
//            dashboardPose = PoseUtils.flipAlliancePose(dashboardPose);
//        }

        field2d.setRobotPose(dashboardPose);
        Logger.getInstance().recordOutput("Odometry/Robot", dashboardPose);
        Logger.getInstance().recordOutput("PoseEstimatorRotation_Rad", dashboardPose.getRotation().getRadians());
        Logger.getInstance().recordOutput("SwerveRotation_Rad", swerve.getRotation2d().getRadians());
    }

    public Notifier getPhotonNotifier() {
        return photonNotifier;
    }
}