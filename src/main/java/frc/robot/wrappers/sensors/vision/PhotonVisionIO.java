package frc.robot.wrappers.sensors.vision;

import org.littletonrobotics.junction.AutoLog;

public interface PhotonVisionIO {
    @AutoLog
    class PhotonVisionIOInputs {
        public double[] pipelineResultTargets = new double[] {};
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see PhotonVisionIOInputsAutoLogged
     * @see AutoLog
     */
    default void updateInputs(final PhotonVisionIOInputsAutoLogged inputs) {}

    default void periodic() {}
}
