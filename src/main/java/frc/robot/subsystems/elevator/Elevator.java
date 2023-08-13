package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.MathUtils;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    protected static final String logKey = "Elevator";

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs;

    private final boolean hasSimSolver;
    private final ElevatorSimSolver elevatorSimSolver;

    private SuperstructureStates.ElevatorState desiredState = SuperstructureStates.ElevatorState.ELEVATOR_RESET;
    private SuperstructureStates.ElevatorState currentState = desiredState;
    private boolean transitioning = false;

    public Elevator(final ElevatorIO elevatorIO, final ElevatorSimSolver elevatorSimSolver) {
        this.elevatorIO = elevatorIO;
        this.inputs = new ElevatorIOInputsAutoLogged();

        this.hasSimSolver = elevatorSimSolver != null;
        this.elevatorSimSolver = elevatorSimSolver;

        setDesiredState(desiredState);
    }

    public Elevator(final ElevatorIO elevatorIO) {
        this(elevatorIO, null);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic() {
        final double elevatorIOPeriodicStart = Logger.getInstance().getRealTimestamp();
        elevatorIO.periodic();

        Logger.getInstance().recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getInstance().getRealTimestamp() - elevatorIOPeriodicStart)
        );

        elevatorIO.updateInputs(inputs);
        Logger.getInstance().processInputs(logKey, inputs);

        final boolean atDesiredState = isAtDesiredState();

        Logger.getInstance().recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.getInstance().recordOutput(logKey + "/DesiredState", desiredState.toString());
        Logger.getInstance().recordOutput(logKey + "/AtDesiredState", atDesiredState);
        Logger.getInstance().recordOutput(logKey + "/IsTransitioning", transitioning);

        Logger.getInstance().recordOutput(logKey + "/VEControlInput", desiredState.getVEControlInput());
        Logger.getInstance().recordOutput(
                logKey + "/VerticalMode", desiredState.getVerticalElevatorMode().toString()
        );

        Logger.getInstance().recordOutput(logKey + "/HEControlInput", desiredState.getHEControlInput());
        Logger.getInstance().recordOutput(
                logKey + "/HorizontalMode", desiredState.getHorizontalElevatorMode().toString()
        );
    }

    /**
     * TODO: document
     * See {@link ElevatorIO#setDesiredState(SuperstructureStates.ElevatorState)}
     */
    public void setDesiredState(final SuperstructureStates.ElevatorState desiredState) {
        this.desiredState = desiredState;
        if (desiredState != currentState) {
            this.transitioning = true;
        }

        elevatorIO.setDesiredState(desiredState);
    }

    public boolean isAtDesiredState() {
        if (currentState == desiredState && !transitioning) {
            return true;
        } else {
            final SuperstructureStates.VerticalElevatorMode verticalElevatorMode = desiredState.getVerticalElevatorMode();
            final SuperstructureStates.HorizontalElevatorMode horizontalElevatorMode = desiredState.getHorizontalElevatorMode();

            final boolean isAtDesired =
                    MathUtils.withinTolerance(
                            switch (verticalElevatorMode) {
                                case POSITION, MOTION_MAGIC -> inputs.verticalElevatorEncoderPosition;
                                case DUTY_CYCLE -> inputs.verticalElevatorMotorDutyCycle;
                            },
                            desiredState.getVEControlInput(),
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (horizontalElevatorMode) {
                                case POSITION -> inputs.horizontalElevatorEncoderPosition;
                                case DUTY_CYCLE -> inputs.horizontalElevatorMotorDutyCycle;
                            },
                            desiredState.getHEControlInput(),
                            0.05
                    );

            if (isAtDesired) {
                this.currentState = desiredState;
                this.transitioning = false;
            }

            return isAtDesired;
        }
    }

    /**
     * Get the current {@link SuperstructureStates.ElevatorState}.
     *
     * <p>Do <b>NOT</b> use this to check if the elevator is currently at all setpoints
     * of the currentState, as this will only report the latest currentState reached by the elevator -
     * i.e. the currentState <i>does <b>not</b> guarantee</i> that the elevator is at <b>all</b> setpoints of that
     * state as it does not take into account the elevator being in a transitional state
     * (transitioning from a previously reached currentState to a new desiredState) where the actual elevator
     * may not be at the setpoint of the (previously reached) currentState anymore.</p>
     *
     * <p>To check if the elevator is at all setpoints of the currentState,
     * use {@link Elevator#isAtState(SuperstructureStates.ElevatorState)}</p>
     *
     * <p>To get a Nullable current state, use {@link Elevator#getCurrentStateWithNullAsTransition()}</p>
     *
     * @return the current {@link SuperstructureStates.ElevatorState}
     * @see SuperstructureStates.ElevatorState
     * @see Elevator#isAtState(SuperstructureStates.ElevatorState)
     * @see Elevator#getCurrentStateWithNullAsTransition()
     */
    public SuperstructureStates.ElevatorState getCurrentState() {
        return currentState;
    }

    /**
     * TODO: document
     * @return TODO
     */
    public SuperstructureStates.ElevatorState getCurrentStateWithNullAsTransition() {
        return transitioning ? null : currentState;
    }

    /**
     * Get if the elevator is currently at a specified {@link SuperstructureStates.ElevatorState}, this takes into
     * account whether the elevator is currently transitioning between states (and reports false if it is)
     * @param elevatorState the {@link SuperstructureStates.ElevatorState} to check against
     * @return true if the elevator is at the specified {@link SuperstructureStates.ElevatorState}, false if not
     * @see SuperstructureStates.ElevatorState
     */
    public boolean isAtState(final SuperstructureStates.ElevatorState elevatorState) {
        return currentState == elevatorState && !transitioning;
    }

    /**
     * Get the desired {@link SuperstructureStates.ElevatorState}
     * @return the currently desired {@link SuperstructureStates.ElevatorState}
     * @see SuperstructureStates.ElevatorState
     */
    public SuperstructureStates.ElevatorState getDesiredState() {
        return desiredState;
    }

    /**
     * Get the {@link ElevatorSimSolver.ElevatorSimState} (simulation state) of the
     * elevator.
     * <p>
     *     The implementation spec requires that this throw an {@link UnsupportedOperationException} or
     *     otherwise a {@link RuntimeException} if an {@link ElevatorSimSolver} does not exist.
     * </p>
     * @return the current {@link ElevatorSimSolver.ElevatorSimState}
     */
    public ElevatorSimSolver.ElevatorSimState getElevatorSimState() {
        if (hasSimSolver) {
            return elevatorSimSolver.getElevatorSimState();
        }

        throw new UnsupportedOperationException("Attempted to GetElevatorSimState without a SimSolver!");
    }

    /**
     * Get whether the vertical part of the elevator is extended upwards
     * @return true if extended, false if not
     */
    public boolean verticalIsExtended() {
        final SuperstructureStates.ElevatorState desiredState = getDesiredState();
        return desiredState == SuperstructureStates.ElevatorState.ELEVATOR_EXTENDED_HIGH
                || desiredState == SuperstructureStates.ElevatorState.ELEVATOR_EXTENDED_MID
                || desiredState == SuperstructureStates.ElevatorState.ELEVATOR_DOUBLE_SUB;
    }
}