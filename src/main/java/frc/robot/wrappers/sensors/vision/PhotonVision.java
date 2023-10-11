package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.vision.TitanCamera;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.*;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class PhotonVision extends VirtualSubsystem {
    protected static final String photonLogKey = "PhotonVision";
//    protected static final String odometryLogKey = "Odometry";

    public static final double TRANSLATION_VELOCITY_TOLERANCE = 0.1;
    public static final double ANGULAR_VELOCITY_TOLERANCE = 0.1;

    public static final AprilTagFieldLayout apriltagFieldLayout;
    public static final AprilTagFieldLayout apriltagFieldAlwaysBlueLayout;

    static {
        AprilTagFieldLayout layout;
        AprilTagFieldLayout alwaysBlueLayout;
        try {
            layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            alwaysBlueLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException ioException) {
            layout = null;
            alwaysBlueLayout = null;
            DriverStation.reportError("Failed to load AprilTagFieldLayout", ioException.getStackTrace());
        }

        apriltagFieldLayout = layout;
        apriltagFieldAlwaysBlueLayout = alwaysBlueLayout;

        if (apriltagFieldLayout != null) {
            apriltagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }

        if (apriltagFieldAlwaysBlueLayout != null) {
            apriltagFieldAlwaysBlueLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    private final List<PhotonVisionIO> photonVisionIOs;
    private final PhotonVisionIOInputsAutoLogged inputs;

    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
//    private final SwerveDriveOdometry visionIndependentOdometry;

    private final List<PhotonRunnable> photonRunnableList;
    private final Map<PhotonRunnable, EstimatedRobotPose> lastEstimatedPosesByCamera;

    private AprilTagFieldLayout.OriginPosition originPosition =
            AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;

    public PhotonVision(
            final List<PhotonVisionIO> photonVisionIOs,
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final SwerveDrivePoseEstimator poseEstimator,
            final List<TitanCamera> apriltagCameras
    ) {
        this.photonVisionIOs = photonVisionIOs;
        this.inputs = new PhotonVisionIOInputsAutoLogged();

        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
//        this.visionIndependentOdometry = visionIndependentOdometry;

        this.photonRunnableList = apriltagCameras
                .stream()
                .map(titanCamera -> new PhotonRunnable(titanCamera, PhotonVision.apriltagFieldAlwaysBlueLayout))
                .toList();
        this.lastEstimatedPosesByCamera = new HashMap<>(apriltagCameras.size());

        refreshAlliance();

        final Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        resetPosition(estimatedPose);
    }

    public static AprilTagFieldLayout.OriginPosition allianceToOrigin(final DriverStation.Alliance alliance) {
        return switch (alliance) {
            case Red -> AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
            case Blue, Invalid -> AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
        };
    }

    public static void refreshAlliance(
            final AprilTagFieldLayout.OriginPosition robotOriginPosition,
            final Consumer<AprilTagFieldLayout.OriginPosition> onAllianceChanged,
            final Consumer<AprilTagFieldLayout.OriginPosition> onRefreshed
    ) {
        final AprilTagFieldLayout.OriginPosition newOriginPosition = allianceToOrigin(DriverStation.getAlliance());
        if (robotOriginPosition != newOriginPosition) {
            onAllianceChanged.accept(newOriginPosition);
        }

        onRefreshed.accept(newOriginPosition);
    }

    public static void refreshAlliance(
            final AprilTagFieldLayout.OriginPosition robotOriginPosition,
            final Consumer<AprilTagFieldLayout.OriginPosition> onAllianceChanged
    ) {
        refreshAlliance(robotOriginPosition, onAllianceChanged, (origin) -> {});
    }

    public enum EstimationRejectionReason {
        DID_NOT_REJECT(0),
        ESTIMATED_POSE_OBJECT_NULL(1),
        ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID(2),
        POSE_NOT_IN_FIELD(3),
        LAST_ESTIMATED_POSE_TIMESTAMP_INVALID_OR_TOO_CLOSE(4),
        POSE_IMPOSSIBLE_VELOCITY(5);

        private final int id;
        EstimationRejectionReason(final int id) {
            this.id = id;
        }

        public int getId() {
            return id;
        }

        public static boolean wasRejected(final EstimationRejectionReason rejectionReason) {
            return rejectionReason != DID_NOT_REJECT;
        }

        public boolean wasRejected() {
            return wasRejected(this);
        }
    }

    public PhotonVision.EstimationRejectionReason shouldRejectEstimation(
            final EstimatedRobotPose lastEstimatedRobotPose,
            final EstimatedRobotPose estimatedRobotPose
    ) {
        if (estimatedRobotPose == null) {
            // reject immediately if the estimated pose itself is null
            return PhotonVision.EstimationRejectionReason.ESTIMATED_POSE_OBJECT_NULL;
        }

        if (estimatedRobotPose.estimatedPose == null
                || estimatedRobotPose.timestampSeconds == -1
                || estimatedRobotPose.targetsUsed.isEmpty()) {
            // reject immediately if null estimatedPose, timestamp is invalid, or no targets used
            return PhotonVision.EstimationRejectionReason.ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID;
        }

        if (lastEstimatedRobotPose == null) {
            // do not reject if there was no last estimation at all (this is different from an invalid last estimation)
            // likely, this is the first time we have an estimation, make sure we accept this estimation
            return PhotonVision.EstimationRejectionReason.DID_NOT_REJECT;
        }

        final Pose3d nextEstimatedPosition = estimatedRobotPose.estimatedPose;
        if (!PoseUtils.isInField(nextEstimatedPosition)) {
            // reject if pose not within the field
            return PhotonVision.EstimationRejectionReason.POSE_NOT_IN_FIELD;
        }

        final double secondsSinceLastUpdate =
                estimatedRobotPose.timestampSeconds - lastEstimatedRobotPose.timestampSeconds;

        if (lastEstimatedRobotPose.timestampSeconds == -1 || secondsSinceLastUpdate <= 0) {
            // TODO: do we always need to reject immediately here? maybe we can still use the next estimation even
            //  if the last estimation had no timestamp or was very close
            return PhotonVision.EstimationRejectionReason.LAST_ESTIMATED_POSE_TIMESTAMP_INVALID_OR_TOO_CLOSE;
        }

        final Pose2d nextEstimatedPosition2d = nextEstimatedPosition.toPose2d();
        final Pose2d lastEstimatedPosition2d = lastEstimatedRobotPose.estimatedPose.toPose2d();
        final Twist2d twist2dToNewEstimation = lastEstimatedPosition2d.log(nextEstimatedPosition2d);

        final double xVel = twist2dToNewEstimation.dx / secondsSinceLastUpdate;
        final double yVel = twist2dToNewEstimation.dy / secondsSinceLastUpdate;
        final double thetaVel = twist2dToNewEstimation.dtheta / secondsSinceLastUpdate;
        final double translationVel = Math.hypot(xVel, yVel);

        Logger.getInstance().recordOutput(
                photonLogKey + "/TranslationVel", translationVel
        );
        Logger.getInstance().recordOutput(
                photonLogKey + "/ThetaVel", thetaVel
        );

        // assume hypot is positive (>= 0)
        if ((translationVel >= Constants.Swerve.ROBOT_MAX_SPEED + PhotonVision.TRANSLATION_VELOCITY_TOLERANCE)
                || (Math.abs(thetaVel) >= Constants.Swerve.ROBOT_MAX_ANGULAR_SPEED + PhotonVision.ANGULAR_VELOCITY_TOLERANCE)) {
            // reject sudden pose changes resulting in an impossible velocity (cannot reach)
            return PhotonVision.EstimationRejectionReason.POSE_IMPOSSIBLE_VELOCITY;
        }

        // TODO: more rejection stuff
        return PhotonVision.EstimationRejectionReason.DID_NOT_REJECT;
    }

    private void update() {
        for (final PhotonRunnable photonRunnable : photonRunnableList) {
            final EstimatedRobotPose lastStableEstimatedRobotPose = photonRunnable.getStableLastEstimatedPose();
            final EstimatedRobotPose estimatedRobotPose = photonRunnable.getLatestEstimatedPose();

            if (estimatedRobotPose == null) {
                // skip the trouble of calling/indexing things if the estimatedRobotPose is null
                continue;
            }

            final EstimatedRobotPose lastSavedEstimatedPose = lastEstimatedPosesByCamera.get(photonRunnable);
            final PhotonVision.EstimationRejectionReason rejectionReason =
                    shouldRejectEstimation(lastSavedEstimatedPose, estimatedRobotPose);

            Logger.getInstance().recordOutput(
                    photonLogKey + "RejectionReason", rejectionReason.getId()
            );

            if (rejectionReason.wasRejected()) {
                continue;
            }

            lastEstimatedPosesByCamera.put(photonRunnable, lastStableEstimatedRobotPose);

            // TODO: get better calibrations on cameras or get better cameras
            poseEstimator.addVisionMeasurement(
                    estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds
            );
        }
    }

    public void logVisionData() {
        final Set<Integer> apriltagIds = new HashSet<>();
        final List<Pose3d> estimatedPoses = new ArrayList<>();
        final List<Pose3d> apriltagPoses = new ArrayList<>();

        for (final EstimatedRobotPose estimatedRobotPose : lastEstimatedPosesByCamera.values()) {
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
                PhotonVision.photonLogKey + "/EstimatedRobotPose3dsByCamera",
                estimatedPoses.toArray(Pose3d[]::new)
        );

        Logger.getInstance().recordOutput(
                PhotonVision.photonLogKey + "/EstimatedRobotPose2dsByCamera",
                estimatedPoses.stream().map(Pose3d::toPose2d).toArray(Pose2d[]::new)
        );

        Logger.getInstance().recordOutput(
                PhotonVision.photonLogKey + "/ApriltagIds",
                apriltagIds.stream().mapToLong(Number::longValue).toArray()
        );

        Logger.getInstance().recordOutput(
                PhotonVision.photonLogKey + "/ApriltagPose3ds",
                apriltagPoses.toArray(Pose3d[]::new)
        );

        Logger.getInstance().recordOutput(
                PhotonVision.photonLogKey + "/ApriltagPose2ds",
                apriltagPoses.stream().map(Pose3d::toPose2d).toArray(Pose2d[]::new)
        );
    }

    @Override
    public void periodic() {
        photonVisionIO.periodic();

        photonVisionIO.updateInputs(inputs);
        Logger.getInstance().processInputs(photonLogKey, inputs);

        // Update and log PhotonVision results
        update();
        logVisionData();

//        final Rotation2d swerveYaw = swerve.getYaw();
//        final SwerveModulePosition[] swerveModulePositions = swerve.getModulePositions();

        // Update PoseEstimator and Odometry
//        final double odometryUpdateStart = Logger.getInstance().getRealTimestamp();
//        visionIndependentOdometry.update(swerveYaw, swerveModulePositions);
//        poseEstimator.update(swerveYaw, swerveModulePositions);
//        final double odometryUpdatePeriodMs = LogUtils.microsecondsToMilliseconds(
//                Logger.getInstance().getRealTimestamp() - odometryUpdateStart
//        );
//
//        final Pose2d estimatedPosition = poseEstimator.getEstimatedPosition();
//        Logger.getInstance().recordOutput(
//                odometryLogKey + "/OdometryUpdatePeriodMs", odometryUpdatePeriodMs
//        );
//        Logger.getInstance().recordOutput(
//                odometryLogKey + "/VisionIndependentRobot2d", visionIndependentOdometry.getPoseMeters()
//        );
//        Logger.getInstance().recordOutput(odometryLogKey + "/Robot2d", estimatedPosition);
//        Logger.getInstance().recordOutput(odometryLogKey + "/Robot3d", GyroUtils.robotPose2dToPose3dWithGyro(
//                estimatedPosition,
//                GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerveYaw)
//        ));
    }

    public void refreshAlliance() {
        refreshAlliance(originPosition, (originPosition) -> {
            final Pose2d estimatedPose = getEstimatedPosition();

            setRobotOriginPosition(originPosition);
            resetPosition(estimatedPose);
        });
    }

    public void setRobotOriginPosition(final AprilTagFieldLayout.OriginPosition robotOriginPosition) {
        apriltagFieldLayout.setOrigin(robotOriginPosition);
        this.originPosition = robotOriginPosition;
    }

    public Pose2d getEstimatedPosition() {
//        return poseEstimator.getEstimatedPosition();
        return new Pose2d();
    }

    public void resetPosition(final Pose2d robotPose, final Rotation2d robotYaw) {
//        visionIndependentOdometry.resetPosition(
//                robotYaw, swerve.getModulePositions(), robotPose
//        );
//
//        poseEstimator.resetPosition(robotYaw, swerve.getModulePositions(), robotPose);
//        photonVisionIO.resetRobotPose(GyroUtils.robotPose2dToPose3dWithGyro(
//                new Pose2d(robotPose.getTranslation(), robotYaw),
//                GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerve.getYaw())
//        ));
    }

    public void resetPosition(final Pose2d robotPose) {
        resetPosition(robotPose, swerve.getYaw());
    }
}
