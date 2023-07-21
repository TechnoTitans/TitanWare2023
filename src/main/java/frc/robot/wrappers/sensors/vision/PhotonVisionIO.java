package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Consumer;

public interface PhotonVisionIO {
    @AutoLog
    class PhotonVisionIOInputs {

    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see PhotonVisionIOInputsAutoLogged
     * @see AutoLog
     */
    void updateInputs(final PhotonVisionIOInputsAutoLogged inputs);

    AprilTagFieldLayout.OriginPosition getRobotOriginPosition();

    void setRobotOriginPosition(final AprilTagFieldLayout.OriginPosition robotOriginPosition);

    default void refreshAlliance(
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

    default void refreshAlliance(
            final AprilTagFieldLayout.OriginPosition robotOriginPosition,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator
    ) {
        refreshAlliance(robotOriginPosition, (originPosition) -> {
            final Pose2d newPose = PoseUtils.flipPose(poseEstimator.getEstimatedPosition());
            poseEstimator.resetPosition(swerve.getYawRotation2d(), swerve.getModulePositions(), newPose);

            setRobotOriginPosition(originPosition);
        });
    }

    void periodic();

    default Pose2d flipPose2dByOriginPosition(
            final Pose2d pose2d, final AprilTagFieldLayout.OriginPosition originPosition
    ) {
        return originPosition == AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
                ? pose2d
                : PoseUtils.flipPose(pose2d);
    }
}
