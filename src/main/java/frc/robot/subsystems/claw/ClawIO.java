package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.utils.Enums;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        String desiredState = Enums.ClawState.CLAW_STANDBY.toString();
        String currentState = desiredState;

        String tiltClawControlMode = Enums.ClawTiltControlMode.POSITION.toString();
        double currentTiltEncoderPositionRots = 0.0;
        double currentTiltEncoderVelocityRotsPerSec = 0.0;
        double desiredTiltControlInput = 0.0;
        double tiltCurrentAmps = 0.0;

        String openCloseControlMode = ControlMode.Position.toString();
        double currentOpenCloseEncoderPositionRots = 0.0;
        double currentOpenCloseEncoderVelocityRotsPerSec = 0.0;
        double desiredOpenCloseControlInput = 0.0;
        double openCloseCurrentAmps = 0.0;

        double desiredIntakeWheelsPercentOutput = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ClawIOInputs
     * @see AutoLog
     */
    void updateInputs(final ClawIOInputs inputs);

    void periodic();

    void config();

    void setDesiredState(final Enums.ClawState state);

    boolean isAtDesiredState();

    Enums.ClawState getDesiredState();

    Enums.ClawState getCurrentState();
}
