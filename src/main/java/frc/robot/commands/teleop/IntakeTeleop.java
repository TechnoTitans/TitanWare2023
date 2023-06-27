package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
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
            if (elevator.getTargetState() == Enums.ElevatorState.ELEVATOR_CUBE) {
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
            }
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
                Commands.sequence(
                        Commands.waitSeconds(0.3),
                        Commands.runOnce(() -> elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY))
                ).schedule();

            }
        } else if (mainController.getStartButton()) {
            elevator.setState(Enums.ElevatorState.ELEVATOR_TIPPED_CONE);
            claw.setState(Enums.ClawState.TIPPED_CONE);
        }


        //Co Controller
        if (coController.getAButton()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_OUTTAKE)),
                    Commands.waitSeconds(0.7),
                    Commands.runOnce(() -> {
                        claw.setState(Enums.ClawState.CLAW_STANDBY);
                        elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
                    })
            ).schedule();
        } else if (coController.getBButton()) {
            elevator.setState(Enums.ElevatorState.SINGLE_SUB);
            claw.setState(Enums.ClawState.SINGLE_SUB);
        } else if (hasLoweredAfter && coController.getRightBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_SHOOT_LOW)),
                    Commands.waitSeconds(0.4),
                    Commands.runOnce(() -> {
                        claw.setState(Enums.ClawState.CLAW_STANDBY);
                        hasLoweredAfter = false;
                    })
            ).schedule();
        } else if (hasLoweredAfter && coController.getLeftBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_SHOOT_HIGH)),
                    Commands.waitSeconds(0.4),
                    Commands.runOnce(() -> {
                        claw.setState(Enums.ClawState.CLAW_STANDBY);
                        hasLoweredAfter = false;
                    })
            ).schedule();
        } else if (coController.getLeftBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_ANGLE_SHOOT)),
                    Commands.waitSeconds(0.5),
                    Commands.runOnce(() -> hasLoweredAfter = true)
            ).schedule();
        } else if (coController.getRightBumper()) {
            Commands.sequence(
                    Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_DROP)),
                    Commands.waitSeconds(0.25),
                    Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_OUTTAKE_HYBIRD)),
                    Commands.waitSeconds(0.65),
                    Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_STANDBY))
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}