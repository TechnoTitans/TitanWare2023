package frc.robot.subsystems.claw;

import frc.robot.utils.Enums;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        double currentTiltPercentOutput = 0.0;
        double currentTiltEncoderPositionRots = 0.0;
        double currentTiltEncoderVelocityRotsPerSec = 0.0;
        double desiredTiltControlInput = 0.0;
        double tiltCurrentAmps = 0.0;

        double currentOpenClosePercentOutput = 0.0;
        double currentOpenCloseEncoderPositionRots = 0.0;
        double currentOpenCloseEncoderVelocityRotsPerSec = 0.0;
        double desiredOpenCloseControlInput = 0.0;
        double openCloseCurrentAmps = 0.0;

        double desiredIntakeWheelsPercentOutput = 0.0;
        double currentIntakeWheelsPercentOutput = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ClawIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ClawIOInputs inputs) {}

    default void periodic() {}

    default void config() {}

    default void setDesiredState(final Enums.ClawState desiredState) {}
}
