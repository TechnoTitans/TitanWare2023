package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.PoseUtils;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Consumer;

public interface PhotonVisionIO {
    @AutoLog
    class PhotonVisionIOInputs {}

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see PhotonVisionIOInputsAutoLogged
     * @see AutoLog
     */
    default void updateInputs(final PhotonVisionIOInputsAutoLogged inputs) {}

    default void periodic() {}

    default void resetRobotPose(final Pose3d robotPose) {}
}
