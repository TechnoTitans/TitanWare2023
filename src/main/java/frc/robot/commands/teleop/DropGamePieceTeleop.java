package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.wrappers.leds.CandleController;

public class DropGamePieceTeleop extends CommandBase {
    private final Claw claw;
    private final Elevator elevator;
    private final CandleController candleController;

    public DropGamePieceTeleop(Claw claw, Elevator elevator, CandleController candleController) {
        this.claw = claw;
        this.elevator = elevator;
        this.candleController = candleController;
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
        candleController.setState(Enums.CANdleState.OFF);
        claw.setState(Enums.ClawState.CLAW_STANDBY);
        elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
    }
}
