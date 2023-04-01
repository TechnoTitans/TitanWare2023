package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;

public class IntakeTeleop extends CommandBase {
    private final Claw claw;
    private final Elevator elevator;
    private final XboxController mainController;
    private final XboxController coController;
    private final Timer timer;
    private final Timer timer2;

    boolean flag = false;
    boolean flag2 = false;
    boolean flag3 = false;

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
        flag3 = false;
    }

    @Override
    public void execute() {

        if (mainController.getAButton()) {
            claw.setState(Enums.ClawState.CLAW_INTAKING_CONE);
        } else if (mainController.getBButton()) {
//            claw.setState(Enums.ClawState.CLAW_INTAKING_CUBE);
            elevator.setState(Enums.ElevatorState.ELEVATOR_CUBE);
            claw.setState(Enums.ClawState.CLAW_ANGLE_CUBE);
        } else if (mainController.getXButton()) {
            claw.setState(Enums.ClawState.CLAW_HOLDING);
            if (elevator.getTargetState() == Enums.ElevatorState.ELEVATOR_CUBE || elevator.getTargetState() == Enums.ElevatorState.SINGLE_SUB) {
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
            }
        }
        else if (coController.getBButton()) {
            elevator.setState(Enums.ElevatorState.SINGLE_SUB);
            claw.setState(Enums.ClawState.SINGLE_SUB);
        }
        else if (coController.getAButton()) {
            if (!flag) {
                timer.reset();
                timer2.reset();
                timer.start();
                flag = true;
            }
            claw.setState(Enums.ClawState.CLAW_DROP);
        }
        else if (coController.getLeftBumper() || coController.getRightBumper()) {
            if (!flag3) {
                timer.reset();
                timer2.reset();
                timer.start();
                flag3 = true;
            }
            claw.setState(Enums.ClawState.CLAW_ANGLE_SHOOT);
        }

        if (flag3) {
            if (timer2.hasElapsed(0.5) && flag2) {
                claw.setState(Enums.ClawState.CLAW_STANDBY);
                flag3 = false;
                flag2 = false;
                timer2.reset();
                timer2.stop();
            }
            else if (coController.getRightBumper() && timer.hasElapsed(.5)){
                claw.setState(Enums.ClawState.CLAW_SHOOT_HIGH);
                flag2 = true;
                timer2.reset();
                timer2.start();
            }
            else if (coController.getLeftBumper() && timer.hasElapsed(.5)){
                claw.setState(Enums.ClawState.CLAW_SHOOT_LOW);
                flag2 = true;
                timer2.reset();
                timer2.start();
            }
        }


        if (flag) {
            if (timer2.hasElapsed(0.5) && flag2) {
                claw.setState(Enums.ClawState.CLAW_STANDBY);
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
                flag = false;
                flag2 = false;
                timer2.reset();
                timer2.stop();
            } else if (coController.getAButton() && timer.hasElapsed(.5)) {
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