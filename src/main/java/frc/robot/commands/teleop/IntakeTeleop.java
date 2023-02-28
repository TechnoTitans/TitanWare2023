package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.wrappers.leds.CandleController;

public class IntakeTeleop extends CommandBase {
    private final Claw claw;
    private final Elevator elevator;
    private final XboxController mainController;
    private final XboxController coController;
    private final Timer timer;
    private final Timer timer2;

    boolean flag = false;
    boolean flag2 = false;

    public IntakeTeleop(Claw claw, Elevator elevator, XboxController mainController, XboxController coController) {
        this.claw = claw;
        this.elevator = elevator;
        this.mainController = mainController;
        this.coController = coController;
        this.timer = new Timer();
        this.timer2 = new Timer();
        CommandScheduler.getInstance().schedule(this);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (mainController.getAButton()) {
            claw.setState(Enums.ClawState.CLAW_INTAKEING);
        } else if (mainController.getXButton()) {
            claw.setState(Enums.ClawState.CLAW_HOLDING);
        } else if (coController.getAButton()) {
            if (!flag) {
                timer.reset();
                timer2.reset();
                timer.start();
                flag = true;
            }
            claw.setState(Enums.ClawState.CLAW_DROP);
        }

        if (flag) {
            if (timer2.hasElapsed(.5) && flag2) {
                claw.setState(Enums.ClawState.CLAW_STANDBY);
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
                flag = false;
                flag2 = false;
                timer2.reset();
                timer2.stop();
            } else if (coController.getAButton() && timer.hasElapsed(1)) {
                claw.setState(Enums.ClawState.CLAW_OUTTAKE);

                flag2 = true;
                timer2.reset();
                timer2.start();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}