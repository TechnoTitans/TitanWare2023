package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.subsystems.ElevatorControl;
import frc.robot.utils.Enums;
import frc.robot.wrappers.api.Slot0Configs;
import frc.robot.wrappers.motors.TitanMAX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerR;
    private final SensorDirectionValue verticalElevatorEncoderR;
    private final TitanMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder;
    private final CANCoder horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch, elevatorHorizontalHighLimitSwitch;

    private Enums.ElevatorState currentState = Enums.ElevatorState.ELEVATOR_RESET;

    public Elevator(
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

        configMotor();

        ElevatorControl elevatorControl = new ElevatorControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, elevatorControl);
    }

    private void configMotor() {
        CANcoderConfiguration CVEConfig = new CANcoderConfiguration();
        CVEConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        CVEConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderR;
        verticalElevatorEncoder.getConfigurator().apply(CVEConfig);

        TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0 = new Slot0Configs(9, 0.15, 0.15925, 1.4126);
        VEConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
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

    public void setState(final Enums.ElevatorState targetState) {
        currentState = targetState;
    }

    public Enums.ElevatorState getTargetState() {
        return currentState;
    }

    public boolean verticalIsExtended() {
        return currentState == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH
            || currentState == Enums.ElevatorState.ELEVATOR_EXTENDED_MID
            || currentState == Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM;
    }

    public TalonFX getVerticalElevatorMotor() {
        return verticalElevatorMotor;
    }

    public CANcoder getVerticalElevatorEncoder() {
        return verticalElevatorEncoder;
    }

    public CANCoder getHorizontalElevatorEncoder() {
        return horizontalElevatorEncoder;
    }

    public TitanMAX getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }

    public DigitalInput getVerticalLimitSwitch() {
        return verticalElevatorLimitSwitch;
    }

    public DigitalInput getHorizontalLimitSwitch() {
        return horizontalElevatorLimitSwitch;
    }

    public DigitalInput getHorizontalHighLimitSwitch() {
        return elevatorHorizontalHighLimitSwitch;
    }

    public double getPosition() {
        return verticalElevatorEncoder.getPosition().refresh().getValue();
    }
}