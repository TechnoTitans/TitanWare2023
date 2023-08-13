package frc.robot.subsystems.elevator;

import frc.robot.utils.SuperstructureStates;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double verticalElevatorMotorDutyCycle = 0.0;
        public double verticalElevatorEncoderPosition = 0.0;
        public double verticalElevatorEncoderVelocity = 0.0;

        public double horizontalElevatorMotorDutyCycle = 0.0;
        public double horizontalElevatorEncoderPosition = 0.0;
        public double horizontalElevatorEncoderVelocity = 0.0;

        public boolean verticalElevatorLimitSwitch = false;
        public boolean horizontalElevatorLimitSwitch = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ElevatorIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ElevatorIOInputs inputs) {}

    /**
     * Periodic call to update the elevator,
     * this could include but isn't limited to updating states and motor setpoints
     */
    default void periodic() {}

    /**
     * Config call, should only be called once
     */
    default void config() {}

    /**
     * Sets the desired state of the elevator to a supplied {@link SuperstructureStates.ElevatorState}
     * @param desiredState the new {@link SuperstructureStates.ElevatorState}
     * @see SuperstructureStates.ElevatorState
     */
    default void setDesiredState(final SuperstructureStates.ElevatorState desiredState) {}
}
