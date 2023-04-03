package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;

public class ElevatorTeleop extends CommandBase {
    private final Elevator elevator;
    private final Claw claw;
    private final XboxController controller;

    public ElevatorTeleop(Elevator elevator, Claw claw, XboxController controller) {
        this.elevator = elevator;
        this.claw = claw;
        this.controller = controller;
    }

    @Override
    public void initialize () {
        elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
    }

    @Override
    public void execute() {
        switch (controller.getPOV()) {
            case 0:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH);
                claw.setState(Enums.ClawState.CLAW_DROP);
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
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
