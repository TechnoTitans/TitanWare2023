package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs;

    public Elevator(final ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        this.inputs = new ElevatorIOInputsAutoLogged();

        config();
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);

        elevatorIO.periodic();
    }

    private void config() {
        elevatorIO.config();
    }

    public void setDesiredState(final Enums.ElevatorState state) {
        elevatorIO.setDesiredState(state);
    }

    public Enums.ElevatorState getDesiredState() {
        return elevatorIO.getDesiredState();
    }

    public ElevatorSimSolver.ElevatorSimState getElevatorSimState() {
        return elevatorIO.getElevatorSimState();
    }

    public boolean verticalIsExtended() {
        return elevatorIO.verticalIsExtended();
    }
}