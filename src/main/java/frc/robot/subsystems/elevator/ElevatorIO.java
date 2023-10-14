package frc.robot.subsystems.elevator;

import frc.robot.utils.SuperstructureStates;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double verticalEncoderPositionRots = 0.0;
        public double verticalEncoderVelocityRotsPerSec = 0.0;
        public double verticalMotorDutyCycle = 0.0;
        public double[] verticalMotorCurrentsAmps = new double[] { 0.0, 0.0 };
        public double[] verticalMotorTempsCelsius = new double[] { 0.0, 0.0 };

        public double horizontalEncoderPositionRots = 0.0;
        public double horizontalEncoderVelocityRotsPerSec = 0.0;
        public double horizontalMotorDutyCycle = 0.0;
        public double horizontalMotorCurrentAmps = 0.0;
        public double horizontalMotorTempCelsius = 0.0;

        public boolean verticalLimitSwitch = false;
        public boolean horizontalLimitSwitch = false;
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
     * Called <b>after</b> {@link ElevatorIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {}

    /**
     * Sets the desired state of the elevator to a supplied {@link SuperstructureStates.ElevatorState}
     * @param desiredState the new {@link SuperstructureStates.ElevatorState}
     * @see SuperstructureStates.ElevatorState
     */
    default void setDesiredState(final SuperstructureStates.ElevatorState desiredState) {}
}
