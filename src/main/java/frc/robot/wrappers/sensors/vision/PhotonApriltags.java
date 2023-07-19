package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.vision.TitanCamera;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class PhotonApriltags extends SubsystemBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;
    private final Map<PhotonRunnable, Notifier> photonRunnableNotifierMap;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition;

    public PhotonApriltags(
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final Field2d field2d,
            final List<TitanCamera> apriltagCameras
    ) {
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.field2d = field2d;

        if (TitanCamera.apriltagFieldLayout == null) {
            this.photonRunnableNotifierMap = Map.of();
            return;
        }

        this.photonRunnableNotifierMap = apriltagCameras
                .stream()
                .map(titanCamera -> new PhotonRunnable(titanCamera, TitanCamera.apriltagFieldLayout))
                .collect(Collectors.toUnmodifiableMap(
                        photonRunnable -> photonRunnable,
                        photonRunnable -> {
                            final Notifier notifier = new Notifier(photonRunnable);
                            notifier.setName(photonRunnable.getPhotonCamera().getName());
                            notifier.startPeriodic(Constants.LOOP_PERIOD_SECONDS);
                            return notifier;
                        }
                ));

        refreshAlliance();
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

    public static Pose2d flipPose2dByOriginPosition(
            final Pose2d pose2d, final AprilTagFieldLayout.OriginPosition originPosition
    ) {
        return originPosition == AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
                ? pose2d
                : PoseUtils.flipPose(pose2d);
    }

    @Override
    public void periodic() {
        for (final PhotonRunnable photonRunnable : photonRunnableNotifierMap.keySet()) {
            final EstimatedRobotPose estimatedRobotPose = photonRunnable.getLatestEstimatedPose();
            if (estimatedRobotPose == null) {
                continue;
            }
            
            //TODO: do we need to do flipPose
            final Pose2d flippedEstimatedPose = flipPose2dByOriginPosition(
                    estimatedRobotPose.estimatedPose.toPose2d(), robotOriginPosition
            );

            //TODO: consider only adding a vision measurement if its somewhat close to the already existing odometry
            poseEstimator.addVisionMeasurement(
                    flippedEstimatedPose,
                    estimatedRobotPose.timestampSeconds
            );
        }

        poseEstimator.update(
                swerve.getYawRotation2d(),
                swerve.getModulePositions()
        );

        final Pose2d estimatedPosition = poseEstimator.getEstimatedPosition();

        field2d.setRobotPose(estimatedPosition);
        Logger.getInstance().recordOutput("Odometry/Robot", estimatedPosition);
    }
}