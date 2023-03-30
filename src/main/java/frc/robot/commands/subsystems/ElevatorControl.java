package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanMAX;

public class ElevatorControl extends CommandBase {
    private final Elevator elevator;
    private final TitanFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch;

    private Enums.ElevatorState currentState;

    private CANSparkMax.ControlType HEControlMode;
    private ControlMode VEControlMode;

    private double
            HETargetRotations = 0, //Horizontal Elevator Target Ticks
            VEPosition = 0; //Vertical Elevator Target Ticks

    private boolean VESwitchFlag = false;

    public ElevatorControl(Elevator elevator) {
        this.elevator = elevator;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        this.verticalElevatorLimitSwitch = elevator.getVerticalLimitSwitch();
        this.horizontalElevatorLimitSwitch = elevator.getHorizontalLimitSwitch();

        addRequirements(elevator);
    }

    private void setState(Enums.ElevatorState state) {
        //TODO TUNE THIS
        this.currentState = state;
        this.VEControlMode = ControlMode.Position;
        switch (state) {
            case ELEVATOR_EXTENDED_HIGH:
                VEPosition = 14500; //15500
                HEControlMode = CANSparkMax.ControlType.kPosition;
                HETargetRotations = 2.5;
                break;
            case ELEVATOR_EXTENDED_MID:
                VEPosition = 11000; //11000
                HEControlMode = CANSparkMax.ControlType.kPosition;
                HETargetRotations = 0.9;
                break;
            case ELEVATOR_STANDBY:
                VEPosition = 50;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.2;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VEPosition = 12500;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.2;
                break;
            case ELEVATOR_CUBE:
                VEPosition = 4000;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.2;
                break;
            case SINGLE_SUB:
                VEPosition = 5500;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.2;
                break;
            default:
                break;
        }
    }

    @Override
    public void execute() {
        Enums.ElevatorState targetState = elevator.getTargetState();
        if (currentState != targetState) {
            setState(targetState);
        }

        if (horizontalElevatorLimitSwitch.get() && targetState == Enums.ElevatorState.ELEVATOR_STANDBY) {
            horizontalElevatorMotor.getAlternateEncoder(8192).setPosition(0);
            HETargetRotations = 0.25;
            HEControlMode = CANSparkMax.ControlType.kPosition;
        }

        if (verticalElevatorLimitSwitch.get() && !VESwitchFlag && targetState == Enums.ElevatorState.ELEVATOR_STANDBY) {
            VESwitchFlag = true;
            verticalElevatorMotor.setSelectedSensorPosition(0);
            VEControlMode = ControlMode.PercentOutput;
            VEPosition = 0;
        } else if (verticalElevatorLimitSwitch.get() && VESwitchFlag && targetState != Enums.ElevatorState.ELEVATOR_STANDBY) {
            VESwitchFlag = false;
        }

        verticalElevatorMotor.set(
                VEControlMode,
                VEPosition);

        horizontalElevatorMotor.set(
                HEControlMode,
                HETargetRotations);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}