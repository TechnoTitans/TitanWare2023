package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.vision.TitanCamera;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;
import java.util.stream.Collectors;

public class PhotonVisionIOApriltagsSim implements PhotonVisionIO {
    private final Swerve swerve;
    private final SwerveDriveOdometry visionIndependentOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final VisionSystemSim visionSystemSim;
    private final List<PhotonCameraSim> photonCameraSims;
    private final Map<PhotonRunnable, Notifier> photonRunnableNotifierMap;

    private final EstimatedRobotPose[] lastEstimatedPosesByCamera;

    public PhotonVisionIOApriltagsSim(
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final SwerveDrivePoseEstimator poseEstimator,
            final List<TitanCamera> apriltagCameras
    ) {
        this.swerve = swerve;
        this.visionIndependentOdometry = visionIndependentOdometry;
        this.poseEstimator = poseEstimator;

        final AprilTagFieldLayout apriltagFieldLayout = PhotonVision.apriltagFieldLayout;
        if (apriltagFieldLayout == null) {
            this.visionSystemSim = null;
            this.photonCameraSims = List.of();
            this.photonRunnableNotifierMap = Map.of();
            this.lastEstimatedPosesByCamera = new EstimatedRobotPose[0];
            return;
        }

        this.visionSystemSim = new VisionSystemSim(PhotonVision.logKey);
        this.visionSystemSim.addVisionTargets(PhotonVision.apriltagFieldLayout);

        this.photonRunnableNotifierMap = apriltagCameras
                .stream()
                .map(titanCamera -> new PhotonRunnable(titanCamera, PhotonVision.apriltagFieldLayout))
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

        this.photonCameraSims = apriltagCameras
                .stream()
                .map(titanCamera -> {
                    final PhotonCameraSim photonCameraSim =
                            new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

                    visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotRelativeToCameraTransform());
                    ToClose.add(photonCameraSim);

                    return photonCameraSim;
                })
                .toList();

        this.lastEstimatedPosesByCamera = new EstimatedRobotPose[apriltagCameras.size()];
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
                    ids.stream().map(
                            id -> PhotonVision.apriltagFieldLayout.getTagPose(id).orElse(new Pose3d())
                    ).toList()
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
                "Vision/ApriltagPose3ds",
                apriltagPoses.toArray(Pose3d[]::new)
        );

        Logger.getInstance().recordOutput(
                "Vision/ApriltagPose2ds",
                apriltagPoses.stream().map(Pose3d::toPose2d).toArray(Pose2d[]::new)
        );
    }

    @Override
    public void periodic() {
        if (ToClose.hasClosed()) {
            // do not try to update if we've already closed
            return;
        }

        final Pose2d visionIndependentPose = visionIndependentOdometry.getPoseMeters();
        visionSystemSim.update(
                GyroUtils.robotPose2dToPose3dWithGyro(
                        visionIndependentPose,
                        GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerve.getYaw())
                )
        );

        for (final ListIterator<PhotonRunnable> runnableIterator = photonRunnableNotifierMap
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

            //TODO: consider only adding a vision measurement if its somewhat close to the already existing odometry
            poseEstimator.addVisionMeasurement(
                    estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds
            );
        }
    }

    @Override
    public void resetRobotPose(final Pose3d robotPose) {
        visionSystemSim.resetRobotPose(robotPose);
    }

}
