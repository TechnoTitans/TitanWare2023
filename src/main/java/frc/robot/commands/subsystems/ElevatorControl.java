package frc.robot.commands.subsystems;

import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanMAX;

public class ElevatorControl extends CommandBase {
    private final Elevator elevator;
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final CANcoder verticalElevatorEncoder;
    private final TitanMAX horizontalElevatorMotor;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch;

    private Enums.ElevatorState currentState;

    private CANSparkMax.ControlType HEControlMode;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;
    private final Follower follower;
    private Enums.ElevatorMode elevatorMode;

    private double
    HETargetRotations = 0, //Horizontal Elevator Target Ticks
    VEPosition = 0; //Vertical Elevator Target Ticks

    private boolean VESwitchFlag = false;

    public ElevatorControl(Elevator elevator) {
        this.elevator = elevator;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.verticalElevatorMotorFollower = elevator.getVerticalElevatorMotorFollower();
        this.verticalElevatorEncoder = elevator.getVerticalElevatorEncoder();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        this.verticalElevatorLimitSwitch = elevator.getVerticalLimitSwitch();
        this.horizontalElevatorLimitSwitch = elevator.getHorizontalLimitSwitch();

        this.positionVoltage = new PositionVoltage(0, true, 0, 0, false);
        this.motionMagicVoltage = new MotionMagicVoltage(0, true, 0, 0, false);
        this.dutyCycleOut = new DutyCycleOut(0, true, false);
        this.follower = new Follower(verticalElevatorMotor.getDeviceID() ,false);

        addRequirements(elevator);
    }

    private void setState(Enums.ElevatorState state) {
        //TODO TUNE THIS
        this.currentState = state;
        this.elevatorMode = Enums.ElevatorMode.POSITION;
        switch (state) {
            case ELEVATOR_EXTENDED_HIGH:
                VEPosition = 5; //15500
                HEControlMode = CANSparkMax.ControlType.kPosition;
                HETargetRotations = 2.5;
                break;
            case ELEVATOR_EXTENDED_MID:
                VEPosition = 3.1; //11000
                HEControlMode = CANSparkMax.ControlType.kPosition;
                HETargetRotations = 0.9;
                break;
            case ELEVATOR_STANDBY:
                elevatorMode = Enums.ElevatorMode.MOTION_MAGIC;
                VEPosition = 0;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.3;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VEPosition = 4.2;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.3;
                break;
            case ELEVATOR_CUBE:
                VEPosition = 1.3;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.3;
                break;
            case SINGLE_SUB:
                VEPosition = 2.1;
                HEControlMode = CANSparkMax.ControlType.kDutyCycle;
                HETargetRotations = -0.3;
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
            verticalElevatorEncoder.setPosition(0);
            elevatorMode = Enums.ElevatorMode.DUTY_CYCLE;
            VEPosition = 0;
        } else if (verticalElevatorLimitSwitch.get() && VESwitchFlag && targetState != Enums.ElevatorState.ELEVATOR_STANDBY) {
            VESwitchFlag = false;
        }

        switch (elevatorMode) {
            case POSITION:
                verticalElevatorMotor.setControl(
                        positionVoltage.withPosition(VEPosition)
                );
                break;
            case MOTION_MAGIC:
                verticalElevatorMotor.setControl(
                        motionMagicVoltage.withPosition(VEPosition)
                );
                break;
            case DUTY_CYCLE:
                verticalElevatorMotor.setControl(
                        dutyCycleOut.withOutput(VEPosition)
                );
                break;
        }

        verticalElevatorMotorFollower.setControl(follower);

        horizontalElevatorMotor.set(
                HEControlMode,
                HETargetRotations
        );
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}