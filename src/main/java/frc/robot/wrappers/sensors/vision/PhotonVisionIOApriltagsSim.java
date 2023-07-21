package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.vision.TitanCamera;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;
import java.util.stream.Collectors;

public class PhotonVisionIOApriltagsSim implements PhotonVisionIO {
    public static final double NO_LED_MAX_LED_RANGE = 1000;

    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final AprilTagFieldLayout aprilTagFieldLayout;
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

        this.aprilTagFieldLayout = TitanCamera.apriltagFieldLayout;
        if (aprilTagFieldLayout == null) {
            this.photonRunnableNotifierMap = Map.of();
            this.cameraSimVisionSystemMap = Map.of();
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


        this.cameraSimVisionSystemMap = apriltagCameras.stream().collect(Collectors.toUnmodifiableMap(
                TitanCamera::photonCamera,
                titanCamera -> {
                    final SimVisionSystem simVisionSystem = new SimVisionSystem(
                        titanCamera.photonCamera().getName(),
                        titanCamera.camDiagonalFOVDeg(),
                        titanCamera.robotRelativeToCameraTransform(),
                        NO_LED_MAX_LED_RANGE,
                        titanCamera.camResolutionWidthPx(),
                        titanCamera.camResolutionHeightPx(),
                        titanCamera.minTargetArea()
                    );

                    simVisionSystem.addVisionTargets(TitanCamera.apriltagFieldLayout);
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
        //TODO: probably use the code in Real instead of this stream stuff, its actually fairly messy and seems like it
        // could be done better with just 1 for loop
        Logger.getInstance().recordOutput(
                "Vision/EstimatedRobotPosesByCamera",
                Arrays.stream(lastEstimatedPosesByCamera)
                    .filter(Objects::nonNull)
                    .map(estimatedRobotPose -> estimatedRobotPose.estimatedPose)
                    .toArray(Pose3d[]::new)
        );

        final Set<Integer> canSeeApriltagIds = Arrays.stream(lastEstimatedPosesByCamera)
                .filter(Objects::nonNull)
                .map(estimatedRobotPose -> estimatedRobotPose.targetsUsed)
                .reduce(
                        new HashSet<>(),
                        (tagIds, photonTrackedTargets) -> {
                            tagIds.addAll(
                                    photonTrackedTargets
                                            .stream()
                                            .map(PhotonTrackedTarget::getFiducialId)
                                            .collect(Collectors.toUnmodifiableSet())
                            );
                            return tagIds;
                        },
                        (tagIds, nextTagIds) -> {
                            tagIds.addAll(nextTagIds);
                            return tagIds;
                        }
                );

        final ArrayList<Pose3d> apriltagPoses = Arrays.stream(lastEstimatedPosesByCamera)
                .filter(Objects::nonNull)
                .map(estimatedRobotPose -> estimatedRobotPose.targetsUsed)
                .reduce(
                        new ArrayList<>(),
                        (tagIds, photonTrackedTargets) -> {
                            tagIds.addAll(
                                    photonTrackedTargets
                                            .stream()
                                            .map(photonTrackedTarget ->
                                                            aprilTagFieldLayout.getTagPose(
                                                                    photonTrackedTarget.getFiducialId()
                                                            ).orElse(new Pose3d())
                                            )
                                            .collect(Collectors.toUnmodifiableSet())
                            );
                            return tagIds;
                        },
                        (tagIds, nextTagIds) -> {
                            tagIds.addAll(nextTagIds);
                            return tagIds;
                        }
                );

        Logger.getInstance().recordOutput(
                "Vision/ApriltagIds",
                canSeeApriltagIds.stream().mapToLong(Number::longValue).toArray()
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
                            new Rotation3d(swerve.getRoll(), swerve.getPitch(), swerve.getYaw()))
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
            final EstimatedRobotPose estimatedRobotPose = photonRunnable.getLatestEstimatedPose();

            lastEstimatedPosesByCamera[index] = estimatedRobotPose;
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
