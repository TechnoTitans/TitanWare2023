package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.safety.SubsystemEStop;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class Elevator extends SubsystemBase {
    protected static final String logKey = "Elevator";

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs;

    private final boolean hasSimSolver;
    private final ElevatorSimSolver elevatorSimSolver;

    private final SubsystemEStop eStop;

    private SuperstructureStates.ElevatorState desiredState = SuperstructureStates.ElevatorState.ELEVATOR_RESET;
    private SuperstructureStates.ElevatorState currentState = desiredState;
    private boolean transitioning = false;

    public Elevator(final ElevatorIO elevatorIO, final ElevatorSimSolver elevatorSimSolver) {
        this.elevatorIO = elevatorIO;
        this.inputs = new ElevatorIOInputsAutoLogged();

        this.hasSimSolver = elevatorSimSolver != null;
        this.elevatorSimSolver = elevatorSimSolver;

        // TODO: test that this stuff works
        //  do we even really need this? reconsider its benefits
        this.eStop = new SubsystemEStop(
                List.of(
//                        SubsystemEStop.StopConditionProvider.instantDoubleLimit(
//                                // TODO: get real lower and upper limits
//                                () -> inputs.verticalElevatorEncoderPosition, -Double.MAX_VALUE, Double.MAX_VALUE
//                        ),
//                        SubsystemEStop.StopConditionProvider.instantDoubleLimit(
//                                // TODO: get real lower and upper limits
//                                () -> inputs.horizontalElevatorEncoderPosition, -Double.MAX_VALUE, Double.MAX_VALUE
//                        ),
//                        new SubsystemEStop.StopConditionProvider<>(
//                                stop -> stop,
//                                new MeasurementObserver(
//                                        () -> getVEControlMeasurement(desiredState.getVerticalElevatorMode()),
//                                        () -> getVEControlVelocity(desiredState.getVerticalElevatorMode()),
//                                        () -> desiredState.getVEControlInput()
//                                )::isAwayFromSetpoint,
//                                new SubsystemEStop.StopBehavior(
//                                        SubsystemEStop.StopBehavior.Kind.AFTER_DURATION, 5
//                                )
//                        )
//                        new SubsystemEStop.StopConditionProvider<>(
//                                stop -> stop,
//                                new MeasurementObserver(
//                                        () -> getHEControlMeasurement(desiredState.getHorizontalElevatorMode()),
//                                        () -> getHEControlVelocity(desiredState.getHorizontalElevatorMode()),
//                                        () -> desiredState.getHEControlInput()
//                                )::isAwayFromSetpoint,
//                                new SubsystemEStop.StopBehavior(
//                                        SubsystemEStop.StopBehavior.Kind.AFTER_DURATION, 5
//                                )
//                        )
                ),
                stopBehavior -> {
                    // TODO: stop throwing here, maybe do some reporting? or maybe throw in non-comp?
                    throw new RuntimeException(stopBehavior.toString());
                },
                // TODO: add manual override?
                () -> false
        );

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

        Logger.getInstance().recordOutput("HorizontalElevatorMotorDutyCycle", inputs.horizontalElevatorMotorDutyCycle);
        Logger.getInstance().recordOutput("HEControlMeasurement", getHEControlMeasurement(desiredState.getHorizontalElevatorMode()));
        Logger.getInstance().recordOutput("HEControlVelocity", getHEControlVelocity(desiredState.getHorizontalElevatorMode()));
        Logger.getInstance().recordOutput("HEControlInput", desiredState.getHEControlInput());

        Logger.getInstance().recordOutput("VEControlMeasurement", getVEControlMeasurement(desiredState.getVerticalElevatorMode()));
        Logger.getInstance().recordOutput("VEControlVelocity", getVEControlVelocity(desiredState.getVerticalElevatorMode()));
        Logger.getInstance().recordOutput("VEControlInput", desiredState.getVEControlInput());

        eStop.periodic();

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
    
    private double getVEControlMeasurement(final SuperstructureStates.VerticalElevatorMode verticalElevatorMode) {
        return switch (verticalElevatorMode) {
            case POSITION, MOTION_MAGIC -> inputs.verticalElevatorEncoderPosition;
            case DUTY_CYCLE -> inputs.verticalElevatorMotorDutyCycle;
        };
    }

    private double getVEControlVelocity(final SuperstructureStates.VerticalElevatorMode verticalElevatorMode) {
        return switch (verticalElevatorMode) {
            case POSITION, MOTION_MAGIC -> inputs.verticalElevatorEncoderVelocity;
            case DUTY_CYCLE -> Math.copySign(Double.MAX_VALUE, inputs.verticalElevatorMotorDutyCycle);
        };
    }

    private double getHEControlMeasurement(final SuperstructureStates.HorizontalElevatorMode horizontalElevatorMode) {
        return switch (horizontalElevatorMode) {
            case POSITION -> inputs.horizontalElevatorEncoderPosition;
            case DUTY_CYCLE -> inputs.horizontalElevatorMotorDutyCycle;
        };
    }

    private double getHEControlVelocity(final SuperstructureStates.HorizontalElevatorMode horizontalElevatorMode) {
        return switch (horizontalElevatorMode) {
            case POSITION -> inputs.horizontalElevatorEncoderVelocity;
            case DUTY_CYCLE -> Math.copySign(Double.MAX_VALUE, inputs.horizontalElevatorMotorDutyCycle);
        };
    }

    /**
     * Sets the desired {@link frc.robot.utils.SuperstructureStates.ElevatorState}.
     *
     * @param desiredState the new desired {@link frc.robot.utils.SuperstructureStates.ElevatorState}
     * @implNote This will put the system into a transitioning state if the new desiredState is != to the currentState
     * @see ElevatorIO#setDesiredState(SuperstructureStates.ElevatorState)
     */
    public void setDesiredState(final SuperstructureStates.ElevatorState desiredState) {
        this.desiredState = desiredState;
        if (desiredState != currentState) {
            this.transitioning = true;
        }

        elevatorIO.setDesiredState(desiredState);
    }

    /**
     * Get if the system is at its desired {@link frc.robot.utils.SuperstructureStates.ElevatorState}.
     *
     * <p>This <b>should</b> be called periodically to update the currentState of the system
     * which will ensure that anything reading from currentState directly without interacting with this method
     * will receive the correct currentState, however, this isn't required if the only interaction with the
     * currentState is through this method (which will update the currentState before returning a result)</p>
     *
     * @return true if the system is at the desired {@link frc.robot.utils.SuperstructureStates.ElevatorState},
     * false if not
     */
    public boolean isAtDesiredState() {
        if (currentState == desiredState && !transitioning) {
            return true;
        } else {
            final boolean isAtDesired = isAtDesiredStateInternal();
            if (isAtDesired) {
                this.currentState = desiredState;
                this.transitioning = false;
            }

            return isAtDesired;
        }
    }

    private boolean isAtDesiredStateInternal() {
        final SuperstructureStates.VerticalElevatorMode verticalElevatorMode =
                desiredState.getVerticalElevatorMode();

        final SuperstructureStates.HorizontalElevatorMode horizontalElevatorMode =
                desiredState.getHorizontalElevatorMode();

        return MathUtils.withinTolerance(
                getVEControlMeasurement(verticalElevatorMode),
                desiredState.getVEControlInput(),
                0.05
        ) && MathUtils.withinTolerance(
                getHEControlMeasurement(horizontalElevatorMode),
                desiredState.getHEControlInput(),
                0.05
        );
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
     * Get the current {@link frc.robot.utils.SuperstructureStates.ElevatorState}, with null serving as the current
     * state if this system is currently transitioning (in a transitory state, i.e. no state).
     *
     * <p>Use {@link Elevator#getCurrentState()} if a null current state is undesirable.</p>
     * @return the current {@link frc.robot.utils.SuperstructureStates.ElevatorState}, which may be null
     * @see Elevator#getCurrentState()
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