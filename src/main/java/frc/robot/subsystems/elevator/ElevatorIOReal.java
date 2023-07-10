package frc.robot.subsystems.elevator;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.Enums;
import frc.robot.wrappers.api.Slot0Configs;
import frc.robot.wrappers.motors.TitanMAX;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerR;
    private final SensorDirectionValue verticalElevatorEncoderR;
    private final TitanMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder;
    private final CANCoder horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch, elevatorHorizontalHighLimitSwitch;

    private Enums.ElevatorState desiredState = Enums.ElevatorState.ELEVATOR_RESET;
    private final ProfiledPIDController horizontalElevatorPID;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;
    private Enums.ElevatorMode verticalElevatorMode;

    private double
            HEPositionRotations = 0, //Horizontal Elevator Target Rotations
            VEPositionRotations = 0; //Vertical Elevator Target Rotations

    private boolean VESwitchFlag = false;
    private boolean horizontalPositionalControl = false;

    public ElevatorIOReal(
            final TalonFX verticalElevatorMotor,
            final InvertedValue verticalElevatorMotorR,
            final TalonFX verticalElevatorMotorFollower,
            final InvertedValue verticalElevatorMotorFollowerR,
            final CANcoder verticalElevatorEncoder,
            final CANCoder horizontalElevatorEncoder,
            final SensorDirectionValue verticalElevatorEncoderR,
            final TitanMAX horizontalElevatorMotor,
            final DigitalInput verticalElevatorLimitSwitch,
            final DigitalInput horizontalElevatorLimitSwitch,
            final DigitalInput elevatorHorizontalHighLimitSwitch
    ) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorR = verticalElevatorMotorR;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerR = verticalElevatorMotorFollowerR;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.verticalElevatorEncoderR = verticalElevatorEncoderR;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;
        this.horizontalElevatorLimitSwitch = horizontalElevatorLimitSwitch;
        this.elevatorHorizontalHighLimitSwitch = elevatorHorizontalHighLimitSwitch;

        config();

        this.horizontalElevatorPID = new ProfiledPIDController(0.3, 0, 0,
                new TrapezoidProfile.Constraints(10, 20));

        this.positionVoltage = new PositionVoltage(
                0, true, 0, 0, false);
        this.motionMagicVoltage = new MotionMagicVoltage(
                0, true, 0, 0, false);
        this.dutyCycleOut = new DutyCycleOut(
                0, true, false);
    }

    private void periodic() {
        if (horizontalElevatorLimitSwitch.get() && desiredState == Enums.ElevatorState.ELEVATOR_STANDBY &&
                horizontalElevatorEncoder.getPosition() < 0.5) {
            horizontalPositionalControl = true;
            HEPositionRotations = 0.1;
        }

        if (horizontalElevatorLimitSwitch.get() && desiredState == Enums.ElevatorState.ELEVATOR_RESET) {
            horizontalElevatorEncoder.setPosition(0);
            horizontalPositionalControl = true;
            HEPositionRotations = 0.1;
        }

        if (elevatorHorizontalHighLimitSwitch.get() && desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH &&
                horizontalElevatorEncoder.getPosition() > 1.5) {
            horizontalPositionalControl = true;
            HEPositionRotations = horizontalElevatorEncoder.getPosition();
        }

        if (horizontalElevatorLimitSwitch.get() && desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM) {
            horizontalPositionalControl = true;
            HEPositionRotations = horizontalElevatorEncoder.getPosition();
        }

        if (verticalElevatorLimitSwitch.get() && !VESwitchFlag &&
                (desiredState == Enums.ElevatorState.ELEVATOR_STANDBY || desiredState == Enums.ElevatorState.ELEVATOR_RESET)) {
            VESwitchFlag = true;
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = Enums.ElevatorMode.DUTY_CYCLE;
            VEPositionRotations = 0;
        } else if (!verticalElevatorLimitSwitch.get() && VESwitchFlag && (desiredState != Enums.ElevatorState.ELEVATOR_STANDBY &&
                desiredState != Enums.ElevatorState.ELEVATOR_RESET)) {
            VESwitchFlag = false;
        }


        switch (verticalElevatorMode) {
            case POSITION:
                verticalElevatorMotor.setControl(
                        positionVoltage.withPosition(VEPositionRotations)
                );
                break;
            case MOTION_MAGIC:
                verticalElevatorMotor.setControl(
                        motionMagicVoltage.withPosition(VEPositionRotations)
                );
                break;
            case DUTY_CYCLE:
                verticalElevatorMotor.setControl(
                        dutyCycleOut.withOutput(VEPositionRotations)
                );
                break;
        }

        if (horizontalPositionalControl) {
            horizontalElevatorMotor.set(
                    CANSparkMax.ControlType.kDutyCycle,
                    horizontalElevatorPID.calculate(horizontalElevatorEncoder.getPosition(), HEPositionRotations)
            );
        } else {
            horizontalElevatorMotor.set(
                    CANSparkMax.ControlType.kDutyCycle,
                    HEPositionRotations
            );
        }
    }

    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        periodic();

        inputs.desiredState = desiredState.name();

        inputs.verticalElevatorMode = verticalElevatorMode.name();
        inputs.VEPositionRotations = VEPositionRotations;

        inputs.horizontalPositionalControl = horizontalPositionalControl;
        inputs.HEPositionRotations = HEPositionRotations;

        inputs.verticalElevatorEncoderPosition = verticalElevatorEncoder.getPosition().refresh().getValue();
        inputs.verticalElevatorEncoderVelocity = verticalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.horizontalElevatorEncoderPosition = horizontalElevatorEncoder.getPosition();
        inputs.horizontalElevatorEncoderVelocity = horizontalElevatorEncoder.getVelocity();

        inputs.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch.get();
        inputs.horizontalElevatorLimitSwitch = horizontalElevatorLimitSwitch.get();
    }

    @Override
    public void config() {
        CANcoderConfiguration CVEConfig = new CANcoderConfiguration();
        CVEConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        CVEConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderR;
        verticalElevatorEncoder.getConfigurator().apply(CVEConfig);

        TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0 = new Slot0Configs(9, 0.15, 0.15925, 1.4126);
        VEConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        VEConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();
        VEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEConfig.MotorOutput.Inverted = verticalElevatorMotorR;
        VEConfig.MotionMagic.MotionMagicCruiseVelocity = 75;
        VEConfig.MotionMagic.MotionMagicAcceleration = 25;
        VEConfig.MotionMagic.MotionMagicJerk = 40;
        verticalElevatorMotor.getConfigurator().apply(VEConfig);

        TalonFXConfiguration VEFConfig = new TalonFXConfiguration();
        VEFConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEFConfig.MotorOutput.Inverted = verticalElevatorMotorFollowerR;
        verticalElevatorMotorFollower.getConfigurator().apply(VEFConfig);

        Follower verticalElevatorFollower = new Follower(
                verticalElevatorMotor.getDeviceID(),
                false
        );
        verticalElevatorMotorFollower.setControl(verticalElevatorFollower);

        horizontalElevatorMotor.brake();

        CANCoderConfiguration horizontalElevatorEncoderConfig = new CANCoderConfiguration();
        horizontalElevatorEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        horizontalElevatorEncoderConfig.unitString = "deg";
        horizontalElevatorEncoderConfig.sensorDirection = true;
        horizontalElevatorEncoderConfig.sensorCoefficient = 1.0 / 4096; // this makes getPosition() return in rotations
        horizontalElevatorEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        horizontalElevatorEncoder.configAllSettings(horizontalElevatorEncoderConfig);
    }

    @Override
    public void setDesiredState(final Enums.ElevatorState state) {
        this.desiredState = state;
        this.verticalElevatorMode = Enums.ElevatorMode.POSITION;
        switch (state) {
            case ELEVATOR_RESET:
                verticalElevatorMode = Enums.ElevatorMode.MOTION_MAGIC;
                VEPositionRotations = -0.25;
                horizontalPositionalControl = false;
                HEPositionRotations = -0.3;
                break;
            case ELEVATOR_EXTENDED_HIGH:
                VEPositionRotations = 5;
                horizontalPositionalControl = true;
                HEPositionRotations = 3;
                break;
            case ELEVATOR_EXTENDED_MID:
                VEPositionRotations = 3.2;
                horizontalPositionalControl = true;
                HEPositionRotations = 0.9;
                break;
            case ELEVATOR_STANDBY:
                verticalElevatorMode = Enums.ElevatorMode.MOTION_MAGIC;
                VEPositionRotations = -0.25;
                horizontalPositionalControl = true;
                HEPositionRotations = 0;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VEPositionRotations = 4.3;
                horizontalPositionalControl = true;
                HEPositionRotations = 0;
                break;
            case ELEVATOR_CUBE:
                VEPositionRotations = 1.3;
                horizontalPositionalControl = false;
                HEPositionRotations = -0.3;
                break;
            case ELEVATOR_TIPPED_CONE:
                VEPositionRotations = 1.55;
                horizontalPositionalControl = true;
                HEPositionRotations = .2;
                break;
            case SINGLE_SUB:
                VEPositionRotations = 2.1;
                horizontalPositionalControl = true;
                HEPositionRotations = 0;
                break;
            default:
                break;
        }
    }

    @Override
    public Enums.ElevatorState getDesiredState() {
        return desiredState;
    }
}
