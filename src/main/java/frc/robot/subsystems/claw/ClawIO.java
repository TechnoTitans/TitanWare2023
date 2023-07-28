package frc.robot.subsystems.claw;

import frc.robot.utils.Enums;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        double tiltPercentOutput = 0.0;
        double tiltEncoderPositionRots = 0.0;
        double tiltEncoderVelocityRotsPerSec = 0.0;
        double tiltCurrentAmps = 0.0;

        double openClosePercentOutput = 0.0;
        double openCloseEncoderPositionRots = 0.0;
        double openCloseEncoderVelocityRotsPerSec = 0.0;
        double openCloseCurrentAmps = 0.0;

        double intakeWheelsPercentOutput = 0.0;
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
