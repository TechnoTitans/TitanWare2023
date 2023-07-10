package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;

public class IntakeTeleop extends CommandBase {
    private final Claw claw;
    private final Elevator elevator;
    private final XboxController mainController;
    private final XboxController coController;

    private boolean hasLoweredAfter = false;

    public IntakeTeleop(
            final Claw claw,
            final Elevator elevator,
            final XboxController mainController,
            final XboxController coController
    ) {
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
            if (elevator.getDesiredState() == Enums.ElevatorState.ELEVATOR_CUBE) {
                elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_STANDBY);
            }
            claw.setDesiredState(Enums.ClawState.CLAW_INTAKING_CONE);
        } else if (mainController.getBButton()) {
            elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_CUBE);
            claw.setDesiredState(Enums.ClawState.CLAW_ANGLE_CUBE);
        } else if (mainController.getXButton()) {
            claw.setDesiredState(Enums.ClawState.CLAW_HOLDING);
            if (elevator.getDesiredState() == Enums.ElevatorState.ELEVATOR_CUBE ||
                    elevator.getDesiredState() == Enums.ElevatorState.SINGLE_SUB ||
                    elevator.getDesiredState() == Enums.ElevatorState.ELEVATOR_TIPPED_CONE) {
                elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_STANDBY);

            } else if (elevator.getDesiredState() == Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM) {
                Commands.sequence(
                        Commands.waitSeconds(0.3),
                        Commands.runOnce(() -> elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_STANDBY))
                ).schedule();

            }
        } else if (mainController.getStartButton()) {
            elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_TIPPED_CONE);
            claw.setDesiredState(Enums.ClawState.TIPPED_CONE);
        }


        //Co Controller
        if (coController.getAButton()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setDesiredState(Enums.ClawState.CLAW_OUTTAKE)),
                    Commands.waitSeconds(0.7),
                    Commands.runOnce(() -> {
                        claw.setDesiredState(Enums.ClawState.CLAW_STANDBY);
                        elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_STANDBY);
                    })
            ).schedule();
        } else if (coController.getBButton()) {
            elevator.setDesiredState(Enums.ElevatorState.SINGLE_SUB);
            claw.setDesiredState(Enums.ClawState.SINGLE_SUB);
        } else if (hasLoweredAfter && coController.getRightBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setDesiredState(Enums.ClawState.CLAW_SHOOT_LOW)),
                    Commands.waitSeconds(0.4),
                    Commands.runOnce(() -> {
                        claw.setDesiredState(Enums.ClawState.CLAW_STANDBY);
                        hasLoweredAfter = false;
                    })
            ).schedule();
        } else if (hasLoweredAfter && coController.getLeftBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setDesiredState(Enums.ClawState.CLAW_SHOOT_HIGH)),
                    Commands.waitSeconds(0.4),
                    Commands.runOnce(() -> {
                        claw.setDesiredState(Enums.ClawState.CLAW_STANDBY);
                        hasLoweredAfter = false;
                    })
            ).schedule();
        } else if (coController.getLeftBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setDesiredState(Enums.ClawState.CLAW_ANGLE_SHOOT)),
                    Commands.waitSeconds(0.5),
                    Commands.runOnce(() -> hasLoweredAfter = true)
            ).schedule();
        } else if (coController.getRightBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setDesiredState(Enums.ClawState.CLAW_DROP)),
                    Commands.waitSeconds(0.25),
                    Commands.runOnce(() -> claw.setDesiredState(Enums.ClawState.CLAW_OUTTAKE_HYBRID)),
                    Commands.waitSeconds(0.65),
                    Commands.runOnce(() -> claw.setDesiredState(Enums.ClawState.CLAW_STANDBY))
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}