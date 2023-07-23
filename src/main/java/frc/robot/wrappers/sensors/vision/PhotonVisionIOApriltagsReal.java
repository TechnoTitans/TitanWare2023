package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.vision.TitanCamera;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;
import java.util.stream.Collectors;

public class PhotonVisionIOApriltagsReal implements PhotonVisionIO {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Map<PhotonRunnable, Notifier> photonRunnableNotifierMap;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition = AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;

    private final EstimatedRobotPose[] lastEstimatedPosesByCamera;

    public PhotonVisionIOApriltagsReal(
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final List<TitanCamera> apriltagCameras
    ) {
        this.poseEstimator = poseEstimator;

        if (TitanCamera.apriltagFieldLayout == null) {
            this.photonRunnableNotifierMap = Map.of();
            this.lastEstimatedPosesByCamera = new EstimatedRobotPose[0];
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

        this.lastEstimatedPosesByCamera = new EstimatedRobotPose[apriltagCameras.size()];

        refreshAlliance(robotOriginPosition, swerve, poseEstimator);
    }

    @Override
    public AprilTagFieldLayout.OriginPosition getRobotOriginPosition() {
        return robotOriginPosition;
    }

    @Override
    public void setRobotOriginPosition(AprilTagFieldLayout.OriginPosition robotOriginPosition) {
        this.robotOriginPosition = robotOriginPosition;
    }

    @Override
    public void updateInputs(final PhotonVisionIOInputsAutoLogged inputs) {
        //TODO: address duplicated code issue (probably by merging sim and real impls)
        final Set<Integer> apriltagIds = new HashSet<>(lastEstimatedPosesByCamera.length);
        final List<Pose3d> estimatedPoses = new ArrayList<>(lastEstimatedPosesByCamera.length);
        final List<Pose3d> apriltagPoses = new ArrayList<>(lastEstimatedPosesByCamera.length);

        for (final EstimatedRobotPose estimatedRobotPose : lastEstimatedPosesByCamera) {
            if (estimatedRobotPose == null) {
                continue;
            }

            final Set<Integer> ids = estimatedRobotPose.targetsUsed.stream()
                    .map(PhotonTrackedTarget::getFiducialId)
                    .collect(Collectors.toSet());

            estimatedPoses.add(estimatedRobotPose.estimatedPose);
            apriltagIds.addAll(ids);
            apriltagPoses.addAll(
                    ids.stream().map(id -> TitanCamera.apriltagFieldLayout.getTagPose(id).orElse(new Pose3d())).toList()
            );
        }

        Logger.getInstance().recordOutput(
                "Vision/EstimatedRobotPosesByCamera",
                estimatedPoses.toArray(Pose3d[]::new)
        );

        Logger.getInstance().recordOutput(
                "Vision/ApriltagIds",
                apriltagIds.stream().mapToLong(Number::longValue).toArray()
        );

        Logger.getInstance().recordOutput(
                "Vision/ApriltagPoses",
                apriltagPoses.toArray(Pose3d[]::new)
        );
    }

    @Override
    public void periodic() {
        for (
                final ListIterator<PhotonRunnable> runnableIterator = photonRunnableNotifierMap
                        .keySet()
                        .stream()
                        .toList()
                        .listIterator();
                runnableIterator.hasNext();
        ) {
            final int index = runnableIterator.nextIndex();
            final PhotonRunnable photonRunnable = runnableIterator.next();

            final EstimatedRobotPose lastStableEstimatedRobotPose = photonRunnable.getStableLastEstimatedPose();
            final EstimatedRobotPose estimatedRobotPose = photonRunnable.getLatestEstimatedPose();

            lastEstimatedPosesByCamera[index] = lastStableEstimatedRobotPose;
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
    }
}