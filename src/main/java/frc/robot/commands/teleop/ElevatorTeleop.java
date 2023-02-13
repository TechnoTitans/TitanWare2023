package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;

public class ElevatorTeleop extends CommandBase {
    private final Elevator elevator;
    private final XboxController controller;

    public ElevatorTeleop(Elevator elevator, XboxController controller) {
        this.elevator = elevator;
        this.controller = controller;
        CommandScheduler.getInstance().schedule(this);
    }

    @Override
    public void execute() {
        switch (controller.getPOV()) {
            case 0:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH);
                break;
            case 90:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_MID);
                break;
            case 180:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_GROUND);
                break;
            case 270:
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}