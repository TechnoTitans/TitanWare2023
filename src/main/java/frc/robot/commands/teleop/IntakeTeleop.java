package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;

public class IntakeTeleop extends CommandBase {
    private final Claw claw;
    private final Elevator elevator;
    private final XboxController mainController;
    private final XboxController coController;

    public IntakeTeleop(Claw claw, Elevator elevator, XboxController mainController, XboxController coController) {
        this.claw = claw;
        this.elevator = elevator;
        this.mainController = mainController;
        this.coController = coController;
        CommandScheduler.getInstance().schedule(this);
    }

    @Override
    public void execute() {

//        if (controller.getBButton()) {
//            claw.setState(Enums.ClawState.CLAW_INTAKING);
//        } else {
//            if (claw.getCurrentGamePiece() != Enums.CurrentGamePiece.NONE) {
//                claw.setState(Enums.ClawState.CLAW_HOLDING);
//            } else {
//                claw.setState(Enums.ClawState.CLAW_STANDBY);
//            }
//        }

        if (mainController.getAButton()) {
            claw.setState(Enums.ClawState.CLAW_INTAKEING);
        } else if (mainController.getXButton()) {
            claw.setState(Enums.ClawState.CLAW_HOLDING);
        } else if (coController.getAButton()) {
            claw.setState(Enums.ClawState.CLAW_INTAKEING);
            new WaitCommand(1.3) {
                @Override
                public void end(boolean interupted) {
                    claw.setState(Enums.ClawState.CLAW_OUTTAKE);
                }
            }.schedule();
            new WaitCommand(2) {
                @Override
                public void end(boolean interupted) {
                    claw.setState(Enums.ClawState.CLAW_STANDBY);
                    elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
                }
            }.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}