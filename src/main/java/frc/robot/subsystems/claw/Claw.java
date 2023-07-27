package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.utils.MathUtils;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    protected static final String logKey = "Claw";

    private final ClawIO clawIO;
    private final ClawIOInputsAutoLogged inputs;

    private Enums.ClawState desiredState = Enums.ClawState.CLAW_STANDBY;
    private Enums.ClawState currentState = desiredState;

    public Claw(final ClawIO clawIO) {
        this.clawIO = clawIO;
        this.inputs = new ClawIOInputsAutoLogged();

        setDesiredState(desiredState);
    }

    @Override
    public void periodic() {
        clawIO.periodic();

        clawIO.updateInputs(inputs);
        Logger.getInstance().processInputs(logKey, inputs);

        final boolean atDesiredState = isAtDesiredState();

        Logger.getInstance().recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.getInstance().recordOutput(logKey + "/DesiredState", desiredState.toString());
        Logger.getInstance().recordOutput(logKey + "/AtDesiredState", atDesiredState);

        Logger.getInstance().recordOutput(
                logKey + "/TiltControlMode", currentState.getClawTiltControlMode().toString()
        );

        Logger.getInstance().recordOutput(
                logKey + "/OpenCloseControlMode", currentState.getClawOpenCloseControlMode().toString()
        );
    }

    public void setDesiredState(final Enums.ClawState desiredState) {
        this.desiredState = desiredState;
        clawIO.setDesiredState(desiredState);
    }

    public boolean isAtDesiredState() {
        if (currentState == desiredState) {
            return true;
        } else {
            final Enums.ClawTiltControlMode tiltControlMode = currentState.getClawTiltControlMode();
            final Enums.ClawOpenCloseControlMode openCloseControlMode = currentState.getClawOpenCloseControlMode();

            final boolean isAtDesired =
                    MathUtils.withinTolerance(
                            inputs.currentIntakeWheelsPercentOutput,
                            inputs.desiredIntakeWheelsPercentOutput,
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (openCloseControlMode) {
                                case POSITION -> inputs.currentOpenCloseEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.currentOpenClosePercentOutput;
                            },
                            inputs.desiredOpenCloseControlInput,
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (tiltControlMode) {
                                case POSITION -> inputs.currentTiltEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.currentTiltPercentOutput;
                            },
                            inputs.desiredTiltControlInput,
                            0.05
                    );

            if (isAtDesired) {
                currentState = desiredState;
            }

            return isAtDesired;
        }
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }
    public Enums.ClawState getDesiredState() {
        return desiredState;
    }
}