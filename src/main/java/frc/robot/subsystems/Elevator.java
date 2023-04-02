package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.*;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.subsystems.ElevatorControl;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanMAX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerR;
    private final SensorDirectionValue verticalElevatorEncoderR;
    private final TitanMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch;

    private Enums.ElevatorState currentState = Enums.ElevatorState.ELEVATOR_STANDBY;

    public Elevator(TalonFX verticalElevatorMotor,
                    InvertedValue verticalElevatorMotorR,
                    TalonFX verticalElevatorMotorFollower,
                    InvertedValue verticalElevatorMotorFollowerR,
                    CANcoder verticalElevatorEncoder,
                    SensorDirectionValue verticalElevatorEncoderR,
                    TitanMAX horizontalElevatorMotor,
                    DigitalInput verticalElevatorLimitSwitch,
                    DigitalInput horizontalElevatorLimitSwitch) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorR = verticalElevatorMotorR;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerR = verticalElevatorMotorFollowerR;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorEncoderR = verticalElevatorEncoderR;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;
        this.horizontalElevatorLimitSwitch = horizontalElevatorLimitSwitch;

        configMotor();

        ElevatorControl elevatorControl = new ElevatorControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, elevatorControl);
    }

    public void telemetry() {
        SmartDashboard.putNumber("Elevator Enc", getPosition());
    }

    private void configMotor() {
        CANcoderConfiguration CVEConfig = new CANcoderConfiguration();
        CVEConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        CVEConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderR;
        verticalElevatorEncoder.getConfigurator().apply(CVEConfig);

        TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0.kP = 9; //.53
        VEConfig.Slot0.kD = 0.15;
        VEConfig.Slot0.kS = 0.15925;
        VEConfig.Slot0.kV = 1.4126;
        VEConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        VEConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();
        VEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEConfig.MotorOutput.Inverted = verticalElevatorMotorR;
        VEConfig.MotionMagic.MotionMagicCruiseVelocity = 75;
        VEConfig.MotionMagic.MotionMagicAcceleration = 50;
        VEConfig.MotionMagic.MotionMagicJerk = 90;
        verticalElevatorMotor.getConfigurator().apply(VEConfig);

        TalonFXConfiguration VEFConfig = new TalonFXConfiguration();
        VEFConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEFConfig.MotorOutput.Inverted = verticalElevatorMotorFollowerR;
        verticalElevatorMotorFollower.getConfigurator().apply(VEFConfig);

        SparkMaxPIDController HEConfig = horizontalElevatorMotor.getPIDController();
        HEConfig.setP(0.18);
        HEConfig.setFeedbackDevice(horizontalElevatorMotor.getAlternateEncoder(8192));
        horizontalElevatorMotor.setOpenLoopRampRate(1);
        horizontalElevatorMotor.brake();
    }

    public void setState(Enums.ElevatorState targetState) {
        currentState = targetState;
    }

//    public boolean isAtWantedState() {
//        return elevatorControl.isAtWantedState();
//    }

    public Enums.ElevatorState getTargetState() {
        return currentState;
    }

    public TalonFX getVerticalElevatorMotor() {
        return verticalElevatorMotor;
    }

    public TalonFX getVerticalElevatorMotorFollower() {
        return verticalElevatorMotorFollower;
    }

    public CANcoder getVerticalElevatorEncoder() {
        return verticalElevatorEncoder;
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

    public double getPosition() {
        return verticalElevatorEncoder.getPosition().getValue();
    }
}