package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

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
