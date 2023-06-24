package frc.robot.commands.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanMAX;

public class ElevatorControl extends CommandBase {
    private final Elevator elevator;
    private final TalonFX verticalElevatorMotor;
    private final CANcoder verticalElevatorEncoder;
    private final TitanMAX horizontalElevatorMotor;
    private final CANCoder horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch, elevatorHorizontalHighLimitSwitch;

    private Enums.ElevatorState currentState;
    private final ProfiledPIDController horizontalElevatorPID;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;
    private Enums.ElevatorMode verticalElevatorMode;

    private double
        HETargetRotations = 0, //Horizontal Elevator Target Ticks
        VEPosition = 0; //Vertical Elevator Target Ticks

    private boolean VESwitchFlag = false;
    private boolean horizontalPositionalControl = false;

    public ElevatorControl(Elevator elevator) {
        this.elevator = elevator;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.verticalElevatorEncoder = elevator.getVerticalElevatorEncoder();
        this.horizontalElevatorEncoder = elevator.getHorizontalElevatorEncoder();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        this.verticalElevatorLimitSwitch = elevator.getVerticalLimitSwitch();
        this.horizontalElevatorLimitSwitch = elevator.getHorizontalLimitSwitch();
        this.elevatorHorizontalHighLimitSwitch = elevator.getHorizontalHighLimitSwitch();

        this.horizontalElevatorPID = new ProfiledPIDController(0.3, 0, 0,
                new TrapezoidProfile.Constraints(10, 20));

        this.positionVoltage = new PositionVoltage(0, true, 0, 0, false);
        this.motionMagicVoltage = new MotionMagicVoltage(0, true, 0, 0, false);
        this.dutyCycleOut = new DutyCycleOut(0, true, false);

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        VESwitchFlag = false;
    }

    private void setState(Enums.ElevatorState state) {
        this.currentState = state;
        this.verticalElevatorMode = Enums.ElevatorMode.POSITION;
        switch (state) {
            case ELEVATOR_RESET:
                verticalElevatorMode = Enums.ElevatorMode.MOTION_MAGIC;
                VEPosition = -0.25;
                horizontalPositionalControl = false;
                HETargetRotations = -0.3;
                break;
            case ELEVATOR_EXTENDED_HIGH:
                VEPosition = 5; //15500
//                horizontalPositionalControl = false;
                horizontalPositionalControl = true;
                HETargetRotations = 3;
//                HETargetRotations = 0.25;
                break;
            case ELEVATOR_EXTENDED_MID:
                VEPosition = 3.2; //11000,
                horizontalPositionalControl = true;
                HETargetRotations = 0.9;
                break;
            case ELEVATOR_STANDBY:
                verticalElevatorMode = Enums.ElevatorMode.MOTION_MAGIC;
                VEPosition = -0.25;
//                horizontalPositionalControl = false;
                horizontalPositionalControl = true;
                HETargetRotations = 0;
//                HETargetRotations = -0.3;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VEPosition = 4.3;
//                horizontalPositionalControl = false;
                horizontalPositionalControl = true;
//                HETargetRotations = -0.3;
                HETargetRotations = 0;
                break;
            case ELEVATOR_CUBE:
                VEPosition = 1.3;
                horizontalPositionalControl = false;
                HETargetRotations = -0.3;
                break;
            case ELEVATOR_TIPPED_CONE:
                VEPosition = 1.55;
                horizontalPositionalControl = true;
                HETargetRotations = .2;
                break;
            case SINGLE_SUB:
                VEPosition = 2.1;
                horizontalPositionalControl = true;
                HETargetRotations = 0;
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

        if (horizontalElevatorLimitSwitch.get() && targetState == Enums.ElevatorState.ELEVATOR_STANDBY &&
                horizontalElevatorEncoder.getPosition() < 0.5) {
//            horizontalElevatorEncoder.setPosition(0);
            horizontalPositionalControl = true;
            HETargetRotations = 0.1;
        }

        if (horizontalElevatorLimitSwitch.get() && targetState == Enums.ElevatorState.ELEVATOR_RESET) {
            horizontalElevatorEncoder.setPosition(0);
            horizontalPositionalControl = true;
            HETargetRotations = 0.1;
        }

        if (elevatorHorizontalHighLimitSwitch.get() && targetState == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH &&
                horizontalElevatorEncoder.getPosition() > 1.5) {
            horizontalPositionalControl = true;
            HETargetRotations = horizontalElevatorEncoder.getPosition();
        }

        if (horizontalElevatorLimitSwitch.get() && targetState == Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM) {
            horizontalPositionalControl = true;
            HETargetRotations = horizontalElevatorEncoder.getPosition();
        }

        if (verticalElevatorLimitSwitch.get() && !VESwitchFlag &&
                (targetState == Enums.ElevatorState.ELEVATOR_STANDBY || targetState == Enums.ElevatorState.ELEVATOR_RESET)) {
            VESwitchFlag = true;
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = Enums.ElevatorMode.DUTY_CYCLE;
            VEPosition = 0;
        } else if (!verticalElevatorLimitSwitch.get() && VESwitchFlag && (targetState != Enums.ElevatorState.ELEVATOR_STANDBY &&
                targetState != Enums.ElevatorState.ELEVATOR_RESET)) {
            VESwitchFlag = false;
        }


        switch (verticalElevatorMode) {
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

        if (horizontalPositionalControl) {
            horizontalElevatorMotor.set(
                CANSparkMax.ControlType.kDutyCycle,
                horizontalElevatorPID.calculate(horizontalElevatorEncoder.getPosition(), HETargetRotations)
            );
        } else {
            horizontalElevatorMotor.set(
                CANSparkMax.ControlType.kDutyCycle,
                HETargetRotations
            );
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}