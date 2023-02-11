package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Enums;

public class IntakeTeleop extends CommandBase {
    private final Claw claw;
    private final XboxController controller;

    public IntakeTeleop(Claw claw, XboxController controller) {
        this.claw = claw;
        this.controller = controller;
        CommandScheduler.getInstance().schedule(this);
    }

    @Override
    public void execute() {
        if (controller.getLeftTriggerAxis() > 0.5) {
            claw.setState(Enums.ClawState.CLAW_INTAKING);
        } else {
            if (claw.getCurrentGamePiece() != Enums.CurrentGamePiece.NONE) {
                claw.setState(Enums.ClawState.CLAW_HOLDING);
            } else {
                claw.setState(Enums.ClawState.CLAW_STANDBY);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}