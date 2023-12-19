package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSimSolver;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.logging.LogUtils;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Claw extends SubsystemBase {
    protected static final String logKey = "Claw";

    private final ClawIO clawIO;
    private final ClawIOInputsAutoLogged inputs;

    private SuperstructureStates.ClawState desiredState = SuperstructureStates.ClawState.CLAW_STANDBY;
    private SuperstructureStates.ClawState currentState = desiredState;
    private boolean transitioning = false;

    public Claw(
            final HardwareConstants.ClawConstants clawConstants,
            final Supplier<Elevator.ElevatorPoseState> elevatorPoseStateSupplier
    ) {
        this.clawIO = switch (Constants.CURRENT_MODE) {
            case REAL -> new ClawIOReal(clawConstants);
            case SIM -> new ClawIOSim(clawConstants, elevatorPoseStateSupplier);
            case REPLAY -> new ClawIO() {};
        };;
        this.inputs = new ClawIOInputsAutoLogged();

        this.clawIO.config();
        this.clawIO.initialize();

        setDesiredState(desiredState);
    }

    @Override
    public void periodic() {
        final double clawIOPeriodicStart = Logger.getRealTimestamp();
        clawIO.periodic();
        clawIO.updateInputs(inputs);

        Logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - clawIOPeriodicStart)
        );

        Logger.processInputs(logKey, inputs);

        final boolean atDesiredState = isAtDesiredState();

        Logger.recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.recordOutput(logKey + "/DesiredState", desiredState.toString());
        Logger.recordOutput(logKey + "/AtDesiredState", atDesiredState);
        Logger.recordOutput(logKey + "/IsTransitioning", transitioning);

        Logger.recordOutput(
                logKey + "/DesiredTiltControlInput", desiredState.getTiltControlInput()
        );
        Logger.recordOutput(
                logKey + "/DesiredOpenCloseControlInput", desiredState.getOpenCloseControlInput()
        );
        Logger.recordOutput(
                logKey + "/DesiredIntakeWheelsPercentOutput", desiredState.getIntakeWheelsPercentOutput()
        );

        Logger.recordOutput(
                logKey + "/DesiredTiltControlMode", desiredState.getClawTiltControlMode().toString()
        );
        Logger.recordOutput(
                logKey + "/DesiredOpenCloseControlMode", desiredState.getClawOpenCloseControlMode().toString()
        );
    }

    /**
     * Sets the desired {@link frc.robot.utils.SuperstructureStates.ClawState}.
     *
     * @param desiredState the new desired {@link frc.robot.utils.SuperstructureStates.ClawState}
     * @implNote This will put the system into a transitioning state if the new desiredState is != to the currentState
     * @see ClawIO#setDesiredState(SuperstructureStates.ClawState)
     */
    public void setDesiredState(final SuperstructureStates.ClawState desiredState) {
        this.desiredState = desiredState;
        if (desiredState != currentState) {
            this.transitioning = true;
        }

        clawIO.setDesiredState(desiredState);
    }

    /**
     * Get if the system is at its desired {@link frc.robot.utils.SuperstructureStates.ClawState}.
     *
     * <p>This <b>should</b> be called periodically to update the currentState of the system
     * which will ensure that anything reading from currentState directly without interacting with this method
     * will receive the correct currentState, however, this isn't required if the only interaction with the
     * currentState is through this method (which will update the currentState before returning a result)</p>
     *
     * @return true if the system is at the desired {@link frc.robot.utils.SuperstructureStates.ClawState},
     * false if not
     */
    public boolean isAtDesiredState() {
        if (currentState == desiredState && !transitioning) {
            return true;
        } else {
            final SuperstructureStates.ClawTiltControlMode tiltControlMode = desiredState.getClawTiltControlMode();
            final SuperstructureStates.ClawOpenCloseControlMode openCloseControlMode = desiredState.getClawOpenCloseControlMode();


            //todo check accuracy on real robot
            final boolean isAtDesired =
                    MathUtils.withinTolerance(
                            inputs.intakeWheelsPercentOutput,
                            desiredState.getIntakeWheelsPercentOutput(),
                            0.15
                    ) && MathUtils.withinTolerance(
                            switch (openCloseControlMode) {
                                case POSITION -> inputs.openCloseEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.openClosePercentOutput;
                            },
                            desiredState.getOpenCloseControlInput(),
                            0.1
                    ) && MathUtils.withinTolerance(
                            switch (tiltControlMode) {
                                case POSITION -> inputs.tiltEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.tiltPercentOutput;
                            },
                            desiredState.getTiltControlInput(),
                            0.1
                    );

            if (isAtDesired) {
                this.currentState = desiredState;
                this.transitioning = false;
            }

            return isAtDesired;
        }
    }

    /**
     * Get the current {@link SuperstructureStates.ClawState}.
     *
     * <p>Do <b>NOT</b> use this to check if the claw is currently at all setpoints
     * of the currentState, as this will only report the latest currentState reached by the claw -
     * i.e. the currentState <i>does <b>not</b> guarantee</i> that the claw is at <b>all</b> setpoints of that
     * state as it does not take into account the claw being in a transitional state
     * (transitioning from a previously reached currentState to a new desiredState) where the actual claw
     * may not be at the setpoint of the (previously reached) currentState anymore.</p>
     *
     * <p>To check if the claw is at all setpoints of the currentState,
     * use {@link Claw#isAtState(SuperstructureStates.ClawState)}</p>
     *
     * <p>To get a Nullable current state, use {@link Claw#getCurrentStateWithNullAsTransition()}</p>
     *
     * @return the current {@link SuperstructureStates.ClawState}
     * @see SuperstructureStates.ClawState
     * @see Claw#isAtState(SuperstructureStates.ClawState)
     * @see Claw#getCurrentStateWithNullAsTransition()
     */
    @SuppressWarnings("unused")
    public SuperstructureStates.ClawState getCurrentState() {
        return currentState;
    }

    /**
     * Get the current {@link frc.robot.utils.SuperstructureStates.ClawState}, with null serving as the current
     * state if this system is currently transitioning (in a transitory state, i.e. no state).
     *
     * <p>Use {@link Claw#getCurrentState()} if a null current state is undesirable.</p>
     * @return the current {@link frc.robot.utils.SuperstructureStates.ClawState}, which may be null
     * @see Claw#getCurrentState()
     */
    public SuperstructureStates.ClawState getCurrentStateWithNullAsTransition() {
        return transitioning ? null : currentState;
    }

    /**
     * Get if the claw is currently at a specified {@link SuperstructureStates.ClawState}, this takes into
     * account whether the claw is currently transitioning between states (and reports false if it is)
     * @param clawState the {@link SuperstructureStates.ClawState} to check against
     * @return true if the claw is at the specified {@link SuperstructureStates.ClawState}, false if not
     * @see SuperstructureStates.ClawState
     */
    public boolean isAtState(final SuperstructureStates.ClawState clawState) {
        return currentState == clawState && !transitioning;
    }

    /**
     * Get the desired {@link SuperstructureStates.ClawState}
     * @return the currently desired {@link SuperstructureStates.ClawState}
     * @see SuperstructureStates.ClawState
     */
    public SuperstructureStates.ClawState getDesiredState() {
        return desiredState;
    }
}