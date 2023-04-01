package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.subsystems.ElevatorControl;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanMAX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TitanFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;
    private final CANCoder verticalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch;

    private Enums.ElevatorState currentState = Enums.ElevatorState.ELEVATOR_EXTENDED_MID;

    public Elevator(TitanFX verticalElevatorMotor,
                    CANCoder verticalElevatorEncoder,
                    TitanMAX horizontalElevatorMotor,
                    DigitalInput verticalElevatorLimitSwitch,
                    DigitalInput horizontalElevatorLimitSwitch) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;
        this.horizontalElevatorLimitSwitch = horizontalElevatorLimitSwitch;

        configMotor();

        ElevatorControl elevatorControl = new ElevatorControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, elevatorControl);
    }

    public void telemetry() {
        SmartDashboard.putNumber("Elevator Enc", getPosition());
        SmartDashboard.putNumber("Elevator Current", getCurrent());
    }

    private void configMotor() {
        CANCoderConfiguration CVEConfig = new CANCoderConfiguration();
        CVEConfig.unitString = "deg";
        CVEConfig.sensorDirection = false;
        CVEConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        verticalElevatorEncoder.configAllSettings(CVEConfig);

        TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.slot0.kP = 0.2; //.53
        VEConfig.slot0.kD = 0.03;
        VEConfig.slot0.kF = 0.045;
        VEConfig.closedloopRamp = 0.2;
        VEConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        VEConfig.remoteFilter0.remoteSensorDeviceID = verticalElevatorEncoder.getDeviceID();
        VEConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        verticalElevatorMotor.configAllSettings(VEConfig);
        verticalElevatorMotor.brake();

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

    public TitanFX getVerticalElevatorMotor() {
        return verticalElevatorMotor;
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
        return verticalElevatorEncoder.getPosition();
    }

    public double getCurrent() {
        return verticalElevatorMotor.getCurrent();
    }

}