package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;

public class ElevatorTeleop extends CommandBase {
    private final Elevator elevator;
    private final Claw claw;
    private final XboxController controller;

    public ElevatorTeleop(
            final Elevator elevator,
            final Claw claw,
            final XboxController controller
    ) {
        this.elevator = elevator;
        this.claw = claw;
        this.controller = controller;
    }

    @Override
    public void initialize () {
        elevator.setState(Enums.ElevatorState.ELEVATOR_RESET);
    }

    @Override
    public void execute() {
        switch (controller.getPOV()) {
            case 0:
                Commands.sequence(
                        Commands.runOnce(() -> elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH)),
                        Commands.waitSeconds(0.3),
                        Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_DROP))
                ).schedule();
                break;
            case 90:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_MID);
                claw.setState(Enums.ClawState.CLAW_DROP);
                break;
            case 180:
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
                claw.setState(Enums.ClawState.CLAW_HOLDING);
                break;
            case 270:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM);
                Commands.sequence(
                        Commands.waitSeconds(0.1),
                        Commands.runOnce(() -> claw.setState(Enums.ClawState.CLAW_INTAKING_CONE))
                ).schedule();
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
