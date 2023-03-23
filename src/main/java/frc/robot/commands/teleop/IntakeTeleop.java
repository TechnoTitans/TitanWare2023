package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
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
        //TODO: THIS IS SHOOT CUBE \/
        else if (coController.getRightBumper()){
            new SequentialCommandGroup(
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_ANGLE_SHOOT)),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_SHOOT_HIGH)),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_STANDBY))
            ).schedule();
        }
        else if (coController.getLeftBumper()){
            new SequentialCommandGroup(
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_ANGLE_SHOOT)),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_SHOOT_LOW)),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_STANDBY))
            ).schedule();
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
        else if (coController.getBButton()) {
            elevator.setState(Enums.ElevatorState.SINGLE_SUB);
            claw.setState(Enums.ClawState.SINGLE_SUB);
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