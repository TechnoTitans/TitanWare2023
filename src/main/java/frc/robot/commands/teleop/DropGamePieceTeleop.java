package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.wrappers.leds.CandleController;

public class DropGamePieceTeleop extends CommandBase {
    private final Claw claw;
    private final Elevator elevator;
    private final CandleController candleController;
    private final XboxController coController;

    public DropGamePieceTeleop(Claw claw, Elevator elevator, CandleController candleController, XboxController coController) {
        this.claw = claw;
        this.elevator = elevator;
        this.candleController = candleController;
        this.coController = coController;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (coController.getXButton()) {
            claw.setState(Enums.ClawState.CLAW_OUTTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
//        candleController.setState(Enums.CANdleState.OFF);
//        claw.setState(Enums.ClawState.CLAW_STANDBY);
//        elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
    }
}
