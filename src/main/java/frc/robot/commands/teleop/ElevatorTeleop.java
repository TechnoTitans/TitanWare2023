package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;

public class ElevatorTeleop extends CommandBase {
    private final Elevator elevator;
    private final XboxController controller;

    public ElevatorTeleop(Elevator elevator, XboxController controller) {
        this.elevator = elevator;
        this.controller = controller;
    }

    @Override
    public void initialize () {
        elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("pov", controller.getPOV());
        switch (controller.getPOV()) {
            case 0:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH);
                break;
            case 90:
                elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_MID);
                break;
            case 180:
                elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY);
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
