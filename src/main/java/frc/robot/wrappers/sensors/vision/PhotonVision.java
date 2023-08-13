package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.gyro.GyroUtils;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.function.Consumer;

public class PhotonVision extends SubsystemBase {
    protected static final String logKey = "PhotonVision";

    public static final AprilTagFieldLayout apriltagFieldLayout;

    static {
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException ioException) {
            layout = null;
            DriverStation.reportError("Failed to load AprilTagFieldLayout", ioException.getStackTrace());
        }

        apriltagFieldLayout = layout;
        if (apriltagFieldLayout != null) {
            apriltagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    private final PhotonVisionIO photonVisionIO;
    private final PhotonVisionIOInputsAutoLogged inputs;
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry visionIndependentOdometry;
    private final Field2d field2d;

    private AprilTagFieldLayout.OriginPosition originPosition;

    public PhotonVision(
            final PhotonVisionIO photonVisionIO,
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final SwerveDrivePoseEstimator poseEstimator,
            final Field2d field2d
    ) {
        this.photonVisionIO = photonVisionIO;
        this.inputs = new PhotonVisionIOInputsAutoLogged();

        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.visionIndependentOdometry = visionIndependentOdometry;

        this.field2d = field2d;

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
            final Consumer<AprilTagFieldLayout.OriginPosition> onAllianceChanged
    ) {
        final AprilTagFieldLayout.OriginPosition newOriginPosition = allianceToOrigin(DriverStation.getAlliance());
        if (robotOriginPosition != newOriginPosition) {
            apriltagFieldLayout.setOrigin(newOriginPosition);
            onAllianceChanged.accept(newOriginPosition);
        }
    }

    @Override
    public void periodic() {
        photonVisionIO.periodic();

        photonVisionIO.updateInputs(inputs);
        Logger.getInstance().processInputs(logKey, inputs);

        final Rotation2d swerveYaw = swerve.getYaw();
        final SwerveModulePosition[] swerveModulePositions = swerve.getModulePositions();

        // Update PoseEstimator and Odometry
        visionIndependentOdometry.update(swerveYaw, swerveModulePositions);
        poseEstimator.update(swerveYaw, swerveModulePositions);

        final Pose2d estimatedPosition = poseEstimator.getEstimatedPosition();

        field2d.setRobotPose(estimatedPosition);
        Logger.getInstance().recordOutput("Odometry/Robot2d", estimatedPosition);
        Logger.getInstance().recordOutput("Odometry/Robot3d", GyroUtils.robotPose2dToPose3dWithGyro(
                estimatedPosition,
                GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerveYaw)
        ));
    }

    public void refreshAlliance() {
        refreshAlliance(originPosition, (originPosition) -> {
            final Pose2d estimatedPose = getEstimatedPosition();

            setRobotOriginPosition(originPosition);
            resetPosition(estimatedPose);
        });
    }

    public void setRobotOriginPosition(final AprilTagFieldLayout.OriginPosition robotOriginPosition) {
        this.originPosition = robotOriginPosition;
    }

    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPosition(final Pose2d robotPose, final Rotation2d robotYaw) {
        visionIndependentOdometry.resetPosition(robotYaw, swerve.getModulePositions(), robotPose);
        poseEstimator.resetPosition(robotYaw, swerve.getModulePositions(), robotPose);
        photonVisionIO.resetRobotPose(new Pose3d(robotPose));
    }

    public void resetPosition(final Pose2d robotPose) {
        resetPosition(robotPose, swerve.getYaw());
    }
}
