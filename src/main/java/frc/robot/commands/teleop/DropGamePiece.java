package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;

import java.util.Timer;

public class DropGamePiece extends CommandBase {
    private final Elevator elevator;
    private final Claw claw;

    public DropGamePiece(Elevator elevator, Claw claw) {
        this.elevator = elevator;
        this.claw = claw;
    }

    @Override
    public void initialize() {
        switch (claw.getCurrentGamePiece()) {
            case CUBE:
                claw.setState(Enums.ClawState.CLAW_OUTTAKE);
                break;
            case CONE:
                claw.setState(Enums.ClawState.CLAW_DROP_CONE);
                break;
            case NONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return claw.getCurrentGamePiece() == Enums.CurrentGamePiece.NONE;
    }

    @Override
    public void end(boolean interrupted) {
        claw.setState(Enums.ClawState.CLAW_HOLDING);
        elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
    }
}
