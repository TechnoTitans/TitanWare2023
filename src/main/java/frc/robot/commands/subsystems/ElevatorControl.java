package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanMAX;

public class ElevatorControl extends CommandBase {
    private final Elevator elevator;
    private final TitanFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;

    private Enums.ElevatorState currentState;

    private double
            HETargetRotations = 0, //Horizontal Elevator Target Ticks
            VEPosition = 0; //Vertical Elevator Target Ticks

    public ElevatorControl(Elevator elevator) {
        this.elevator = elevator;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();

        addRequirements(elevator);
    }

    private void setState(Enums.ElevatorState state) {
        //TODO TUNE THIS
        switch (state) {
            case ELEVATOR_EXTENDED_HIGH:
                VEPosition = 16000;
                HETargetRotations = 3;
                break;
            case ELEVATOR_EXTENDED_MID:
                VEPosition = 10000;
                HETargetRotations = 1.7;
                break;
            case ELEVATOR_EXTENDED_GROUND:
                VEPosition = 1;
                HETargetRotations = 0;
                break;
            case ELEVATOR_STANDBY:
                VEPosition = 50;
                HETargetRotations = 0;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VEPosition = 50;
                HETargetRotations = 15;
                break;
            case ELEVATOR_PREGAME:
                HETargetRotations = 0;
                VEPosition = 0;
                break;
            default:
                break;
        }
    }

    public boolean isAtWantedState() {
        return MathMethods.withinRange(
                verticalElevatorMotor.getSelectedSensorPosition(),
                VEPosition,
                0.1) &&
                MathMethods.withinRange(
                        horizontalElevatorMotor.getAlternateEncoder(8192).getPosition(),
                        HETargetRotations,
                        0.1);
    }

    @Override
    public void execute() {
        Enums.ElevatorState targetState = elevator.getTargetState();
        if (targetState != currentState) {
            currentState = targetState;
            setState(currentState);
        }

        verticalElevatorMotor.set(
                ControlMode.Position,
                VEPosition);

        horizontalElevatorMotor.set(
                CANSparkMax.ControlType.kPosition,
                HETargetRotations);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}