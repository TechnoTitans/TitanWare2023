package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.MathUtils;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    protected static final String logKey = "Claw";

    private final ClawIO clawIO;
    private final ClawIOInputsAutoLogged inputs;

    private SuperstructureStates.ClawState desiredState = SuperstructureStates.ClawState.CLAW_STANDBY;
    private SuperstructureStates.ClawState currentState = desiredState;
    private boolean transitioning = false;

    public Claw(final ClawIO clawIO) {
        this.clawIO = clawIO;
        this.inputs = new ClawIOInputsAutoLogged();

        setDesiredState(desiredState);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic() {
        final double clawIOPeriodicStart = Logger.getInstance().getRealTimestamp();
        clawIO.periodic();

        Logger.getInstance().recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getInstance().getRealTimestamp() - clawIOPeriodicStart)
        );

        clawIO.updateInputs(inputs);
        Logger.getInstance().processInputs(logKey, inputs);

        final boolean atDesiredState = isAtDesiredState();

        Logger.getInstance().recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.getInstance().recordOutput(logKey + "/DesiredState", desiredState.toString());
        Logger.getInstance().recordOutput(logKey + "/AtDesiredState", atDesiredState);
        Logger.getInstance().recordOutput(logKey + "/IsTransitioning", transitioning);

        Logger.getInstance().recordOutput(
                logKey + "/DesiredTiltControlInput", desiredState.getTiltControlInput()
        );
        Logger.getInstance().recordOutput(
                logKey + "/DesiredOpenCloseControlInput", desiredState.getOpenCloseControlInput()
        );
        Logger.getInstance().recordOutput(
                logKey + "/DesiredIntakeWheelsPercentOutput", desiredState.getIntakeWheelsPercentOutput()
        );

        Logger.getInstance().recordOutput(
                logKey + "/DesiredTiltControlMode", desiredState.getClawTiltControlMode().toString()
        );
        Logger.getInstance().recordOutput(
                logKey + "/DesiredOpenCloseControlMode", desiredState.getClawOpenCloseControlMode().toString()
        );
    }

    public void setDesiredState(final SuperstructureStates.ClawState desiredState) {
        this.desiredState = desiredState;
        if (desiredState != currentState) {
            this.transitioning = true;
        }

        clawIO.setDesiredState(desiredState);
    }

    public boolean isAtDesiredState() {
        if (currentState == desiredState && !transitioning) {
            return true;
        } else {
            final SuperstructureStates.ClawTiltControlMode tiltControlMode = desiredState.getClawTiltControlMode();
            final SuperstructureStates.ClawOpenCloseControlMode openCloseControlMode = desiredState.getClawOpenCloseControlMode();

            final boolean isAtDesired =
                    MathUtils.withinTolerance(
                            inputs.intakeWheelsPercentOutput,
                            desiredState.getIntakeWheelsPercentOutput(),
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (openCloseControlMode) {
                                case POSITION -> inputs.openCloseEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.openClosePercentOutput;
                            },
                            desiredState.getOpenCloseControlInput(),
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (tiltControlMode) {
                                case POSITION -> inputs.tiltEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.tiltPercentOutput;
                            },
                            desiredState.getTiltControlInput(),
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
     * TODO: document
     * @return TODO
     */
    public SuperstructureStates.ClawState getCurrentState() {
        return currentState;
    }

    /**
     * TODO: document
     * @return TODO
     */
    public SuperstructureStates.ClawState getCurrentStateWithNullAsTransition() {
        return transitioning ? null : currentState;
    }

    /**
     * TODO: document
     * @param clawState TODO
     * @return TODO
     */
    public boolean isAtState(final SuperstructureStates.ClawState clawState) {
        return currentState == clawState && !transitioning;
    }

    /**
     * TODO: document
     * @return TODO
     */
    public SuperstructureStates.ClawState getDesiredState() {
        return desiredState;
    }
}