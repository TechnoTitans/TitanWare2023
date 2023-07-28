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
                            inputs.intakeWheelsPercentOutput,
                            currentState.getIntakeWheelsPercentOutput(),
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (openCloseControlMode) {
                                case POSITION -> inputs.openCloseEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.openClosePercentOutput;
                            },
                            currentState.getOpenCloseControlInput(),
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (tiltControlMode) {
                                case POSITION -> inputs.tiltEncoderPositionRots;
                                case DUTY_CYCLE -> inputs.tiltPercentOutput;
                            },
                            currentState.getTiltControlInput(),
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