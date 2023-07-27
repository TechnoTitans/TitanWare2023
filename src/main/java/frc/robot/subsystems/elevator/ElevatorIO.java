package frc.robot.subsystems.elevator;

import frc.robot.utils.Enums;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        /**
         * Vertical Elevator Control Input
         *
         * <p>The units for this input are dependent on the
         * {@link frc.robot.utils.Enums.VerticalElevatorMode} currently set.</p>
         *
         * <p>When {@link frc.robot.utils.Enums.VerticalElevatorMode} is
         * {@link frc.robot.utils.Enums.VerticalElevatorMode#POSITION},
         * the units for this control input are in position rotations.</p>
         *
         * @see frc.robot.utils.Enums.VerticalElevatorMode
         */
        public double VEControlInput = 0.0;

        /**
         * Horizontal Elevator Control Input
         *
         * <p>The units for this input are dependent on the
         * {@link frc.robot.utils.Enums.HorizontalElevatorMode} currently set.</p>
         *
         * <p>When {@link frc.robot.utils.Enums.HorizontalElevatorMode} is
         * {@link frc.robot.utils.Enums.HorizontalElevatorMode#POSITION},
         * the units for this control input are in position rotations.</p>
         *
         * @see frc.robot.utils.Enums.HorizontalElevatorMode
         */
        public double HEControlInput = 0.0;

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
     * Sets the desired state of the elevator to a supplied {@link frc.robot.utils.Enums.ElevatorState}
     * @param desiredState the new {@link frc.robot.utils.Enums.ElevatorState}
     * @see frc.robot.utils.Enums.ElevatorState
     */
    default void setDesiredState(final Enums.ElevatorState desiredState) {}
}
