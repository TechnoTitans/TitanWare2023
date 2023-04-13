package frc.robot.commands.teleop;

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

    private boolean hasLoweredAfter = false;

    public IntakeTeleop(Claw claw, Elevator elevator, XboxController mainController, XboxController coController) {
        this.claw = claw;
        this.elevator = elevator;
        this.mainController = mainController;
        this.coController = coController;
        CommandScheduler.getInstance().schedule(this);
    }

    @Override
    public void initialize() {
        hasLoweredAfter = false;
    }

    @Override
    public void execute() {
        //Main Controller
        if (mainController.getAButton()) {
            claw.setState(Enums.ClawState.CLAW_INTAKING_CONE);
        } else if (mainController.getBButton()) {
            elevator.setState(Enums.ElevatorState.ELEVATOR_CUBE);
            claw.setState(Enums.ClawState.CLAW_ANGLE_CUBE);
        } else if (mainController.getXButton()) {
            claw.setState(Enums.ClawState.CLAW_HOLDING);
            if (elevator.getTargetState() == Enums.ElevatorState.ELEVATOR_CUBE ||
                    elevator.getTargetState() == Enums.ElevatorState.SINGLE_SUB ||
                    elevator.getTargetState() == Enums.ElevatorState.ELEVATOR_TIPPED_CONE) {
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);

            } else if (elevator.getTargetState() == Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM) {
                new SequentialCommandGroup(
                        new WaitCommand(0.3),
                        new InstantCommand(() -> elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY))
                ).schedule();

            }
        } else if (mainController.getStartButton()) {
            elevator.setState(Enums.ElevatorState.ELEVATOR_TIPPED_CONE);
            claw.setState(Enums.ClawState.TIPPED_CONE);
        }


        //Co Controller
        if (coController.getAButton()) {
            new SequentialCommandGroup(
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_OUTTAKE)),
                    new WaitCommand(0.7),
                    new InstantCommand(() -> {
                        claw.setState(Enums.ClawState.CLAW_STANDBY);
                        elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
                    })
            ).schedule();
        } else if (coController.getBButton()) {
            elevator.setState(Enums.ElevatorState.SINGLE_SUB);
            claw.setState(Enums.ClawState.SINGLE_SUB);
        } else if (hasLoweredAfter && coController.getRightBumper()) {
            new SequentialCommandGroup(
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_SHOOT_LOW)),
                    new WaitCommand(0.4),
                    new InstantCommand(() -> {
                        claw.setState(Enums.ClawState.CLAW_STANDBY);
                        hasLoweredAfter = false;
                    })
            ).schedule();
        } else if (hasLoweredAfter && coController.getLeftBumper()) {
            new SequentialCommandGroup(
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_SHOOT_HIGH)),
                    new WaitCommand(0.4),
                    new InstantCommand(() -> {
                        claw.setState(Enums.ClawState.CLAW_STANDBY);
                        hasLoweredAfter = false;
                    })
            ).schedule();
        } else if (coController.getLeftBumper()) {
            new SequentialCommandGroup(
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_ANGLE_SHOOT)),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> hasLoweredAfter = true)
            ).schedule();
        } else if (coController.getRightBumper()) {
            new SequentialCommandGroup(
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_DROP)),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_OUTTAKE_HYBIRD)),
                    new WaitCommand(0.65),
                    new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_STANDBY))
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}