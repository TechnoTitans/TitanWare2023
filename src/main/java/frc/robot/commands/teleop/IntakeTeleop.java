package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Enums;

public class IntakeTeleop extends CommandBase {
    private final Claw claw;
    private final XboxController controller;
    private final XboxController controller2;

    public IntakeTeleop(Claw claw, XboxController controller, XboxController controller2) {
        this.claw = claw;
        this.controller = controller;
        this.controller2 = controller2;
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

        if (controller.getBButton()) {
            claw.setState(Enums.ClawState.CLAW_INTAKE_CONE);
        } else if (controller.getAButton()) {
            claw.setState(Enums.ClawState.CLAW_INTAKE_CUBE);
        } else if (controller.getXButton()) {
            claw.setState(Enums.ClawState.CLAW_HOLDING);
        } else if (controller2.getXButton()) {
            claw.setState(Enums.ClawState.CLAW_OUTTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}