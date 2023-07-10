package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;

public class Claw extends SubsystemBase {
    private final ClawIO clawIO;
    private final ClawIOInputsAutoLogged inputs;

    public Claw(final ClawIO clawIO) {
        this.clawIO = clawIO;
        this.inputs = new ClawIOInputsAutoLogged();

        config();
    }

    @Override
    public void periodic() {
        clawIO.updateInputs(inputs);
    }

    private void config() {
        clawIO.config();
    }

    public void setDesiredState(final Enums.ClawState state) {
        clawIO.setDesiredState(state);
    }

    public boolean isAtDesiredState() {
        return clawIO.isAtDesiredState();
    }

    public Enums.ClawState getCurrentState() {
        return clawIO.getCurrentState();
    }
    public Enums.ClawState getDesiredState() {
        return clawIO.getDesiredState();
    }
}