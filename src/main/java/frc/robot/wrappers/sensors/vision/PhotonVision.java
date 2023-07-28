package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class PhotonVision extends SubsystemBase {
    private final PhotonVisionIO photonVisionIO;
    private final PhotonVisionIOInputsAutoLogged inputs;
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;

    private AprilTagFieldLayout.OriginPosition originPosition;

    public PhotonVision(
            final PhotonVisionIO photonVisionIO,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final Field2d field2d
    ) {
        this.photonVisionIO = photonVisionIO;
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.field2d = field2d;

        this.inputs = new PhotonVisionIOInputsAutoLogged();

        refreshAlliance(getRobotOriginPosition(), swerve, poseEstimator);
    }

    @Override
    public void periodic() {
        photonVisionIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        photonVisionIO.periodic();

        poseEstimator.update(
                swerve.getYaw(),
                swerve.getModulePositions()
        );

        final Pose2d estimatedPosition = poseEstimator.getEstimatedPosition();

        field2d.setRobotPose(estimatedPosition);
        Logger.getInstance().recordOutput("Odometry/Robot", estimatedPosition);
    }

    public void refreshAlliance(
            final AprilTagFieldLayout.OriginPosition robotOriginPosition,
            final Consumer<AprilTagFieldLayout.OriginPosition> onAllianceChanged
    ) {
        final boolean allianceChanged;
        final AprilTagFieldLayout.OriginPosition newOriginPosition;
        switch (DriverStation.getAlliance()) {
            case Blue -> {
                allianceChanged = (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
                newOriginPosition = AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
            }
            case Red -> {
                allianceChanged = (robotOriginPosition == AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
                newOriginPosition = AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
            }
            default -> {
                return;
            }
        }

        if (allianceChanged) {
            onAllianceChanged.accept(newOriginPosition);
        }
    }

    public void refreshAlliance(
            final AprilTagFieldLayout.OriginPosition robotOriginPosition,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator
    ) {
        refreshAlliance(robotOriginPosition, (originPosition) -> {
            final Pose2d newPose = PoseUtils.flipPose(poseEstimator.getEstimatedPosition());
            poseEstimator.resetPosition(swerve.getYaw(), swerve.getModulePositions(), newPose);

            setRobotOriginPosition(originPosition);
        });
    }

    public void setRobotOriginPosition(final AprilTagFieldLayout.OriginPosition robotOriginPosition) {
        this.originPosition = robotOriginPosition;
        photonVisionIO.setRobotOriginPosition(robotOriginPosition);
    }

    public AprilTagFieldLayout.OriginPosition getRobotOriginPosition() {
        return originPosition;
    }
}
