package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.vision.TitanCamera;
import frc.robot.wrappers.sensors.vision.replacements.SimTitanVisionSystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;
import java.util.stream.Collectors;

public class PhotonVisionIOApriltagsSim implements PhotonVisionIO {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Map<PhotonCamera, SimVisionSystem> cameraSimVisionSystemMap;
    private final Map<PhotonRunnable, Notifier> photonRunnableNotifierMap;
    private AprilTagFieldLayout.OriginPosition robotOriginPosition;

    private final EstimatedRobotPose[] lastEstimatedPosesByCamera;

    public PhotonVisionIOApriltagsSim(
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final List<TitanCamera> apriltagCameras
    ) {
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;

        final AprilTagFieldLayout apriltagFieldLayout = TitanCamera.apriltagFieldLayout;
        if (apriltagFieldLayout == null) {
            this.photonRunnableNotifierMap = Map.of();
            this.cameraSimVisionSystemMap = Map.of();
            this.lastEstimatedPosesByCamera = new EstimatedRobotPose[0];
            return;
        }

        this.photonRunnableNotifierMap = apriltagCameras
                .stream()
                .map(titanCamera -> new PhotonRunnable(titanCamera, apriltagFieldLayout))
                .collect(Collectors.toUnmodifiableMap(
                        photonRunnable -> photonRunnable,
                        photonRunnable -> {
                            final Notifier notifier = new Notifier(photonRunnable);
                            notifier.setName(photonRunnable.getPhotonCamera().getName());
                            notifier.startPeriodic(Constants.LOOP_PERIOD_SECONDS);

                            // add notifiers to close hook
                            ToClose.add(notifier);

                            return notifier;
                        }
                ));


        this.cameraSimVisionSystemMap = apriltagCameras.stream().collect(Collectors.toUnmodifiableMap(
                TitanCamera::getPhotonCamera,
                titanCamera -> {
                    final SimTitanVisionSystem simVisionSystem = new SimTitanVisionSystem(titanCamera);

                    simVisionSystem.addVisionTargets(apriltagFieldLayout);
                    return simVisionSystem;
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
        //TODO: address duplicated code (probably by merging sim and real when sim is finished and working)
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
        final Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        for (final SimVisionSystem simVisionSystem : cameraSimVisionSystemMap.values()) {
            simVisionSystem.processFrame(
                    new Pose3d(estimatedPose.getX(), estimatedPose.getY(), 0,
                            GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerve.getYaw())
                    )
            );
        }

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
//            poseEstimator.addVisionMeasurement(
//                    flippedEstimatedPose,
//                    estimatedRobotPose.timestampSeconds
//            );
        }
    }
}
