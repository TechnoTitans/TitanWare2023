package frc.robot.subsystems.elevator;

import frc.robot.utils.Enums;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public String desiredState = Enums.ElevatorState.ELEVATOR_RESET.toString();
        public String verticalElevatorMode = Enums.VerticalElevatorMode.MOTION_MAGIC.toString();
        /**
         * Vertical Elevator Control Input
         * <p>The units for this input are dependent on the {@link #verticalElevatorMode} currently set.</p>
         * <p>When {@link #verticalElevatorMode} is {@link frc.robot.utils.Enums.VerticalElevatorMode#POSITION},
         * the units for this control input are in position rotations.</p>
         * @see #verticalElevatorMode
         * @see frc.robot.utils.Enums.VerticalElevatorMode
         */
        public double VEControlInput = 0.0;
        public String horizontalElevatorMode = Enums.HorizontalElevatorMode.DUTY_CYCLE.toString();
        /**
         * Horizontal Elevator Control Input
         * <p>The units for this input are dependent on the {@link #horizontalElevatorMode} currently set.</p>
         * <p>When {@link #horizontalElevatorMode} is {@link frc.robot.utils.Enums.HorizontalElevatorMode#POSITION},
         * the units for this control input are in position rotations.</p>
         * @see #horizontalElevatorMode
         * @see frc.robot.utils.Enums.HorizontalElevatorMode
         */
        public double HEControlInput = 0.0;

        public double verticalElevatorEncoderPosition = 0.0;
        public double verticalElevatorEncoderVelocity = 0.0;
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
    void updateInputs(final ElevatorIOInputs inputs);

    /**
     * Get if the current {@link ElevatorIO} is real
     * @return true if real, false if not
     */
    boolean isReal();

    /**
     * Periodic call to update the elevator,
     * this could include but isn't limited to updating states and motor setpoints
     */
    void periodic();

    /**
     * Config call, should only be called once
     */
    void config();

    /**
     * Sets the desired state of the elevator to a supplied {@link frc.robot.utils.Enums.ElevatorState}
     * @param state the new {@link frc.robot.utils.Enums.ElevatorState}
     * @see frc.robot.utils.Enums.ElevatorState
     */
    void setDesiredState(final Enums.ElevatorState state);

    /**
     * Get the desired {@link frc.robot.utils.Enums.ElevatorState}
     * @return the currently desired {@link frc.robot.utils.Enums.ElevatorState}
     * @see frc.robot.utils.Enums.ElevatorState
     */
    Enums.ElevatorState getDesiredState();

    /**
     * Get the {@link frc.robot.subsystems.elevator.ElevatorSimSolver.ElevatorSimState} (simulation state) of the
     * elevator.
     * <p>
     *     The implementation spec requires that this throw an {@link UnsupportedOperationException} or
     *     otherwise a {@link RuntimeException} if not in sim.
     * </p>
     * @return the current {@link frc.robot.subsystems.elevator.ElevatorSimSolver.ElevatorSimState}
     */
    ElevatorSimSolver.ElevatorSimState getElevatorSimState();

    /**
     * Get whether the vertical part of the elevator is extended upwards
     * @return true if extended, false if not
     */
    default boolean verticalIsExtended() {
        final Enums.ElevatorState desiredState = getDesiredState();
        return desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH
                || desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_MID
                || desiredState == Enums.ElevatorState.ELEVATOR_DOUBLE_SUB;
    }
}
