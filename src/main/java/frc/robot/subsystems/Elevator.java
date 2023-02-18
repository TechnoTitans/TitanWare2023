package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.subsystems.ElevatorControl;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TitanFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;
    private final TitanSRX encoderSRX;

    private final ElevatorControl elevatorControl;
    private Enums.ElevatorState currentState = Enums.ElevatorState.ELEVATOR_STANDBY;

    public Elevator(TitanFX verticalElevatorMotor,
                    TitanMAX horizontalElevatorMotor,
                    TitanSRX encoderSRX) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.encoderSRX = encoderSRX;

        configMotor();

        this.elevatorControl = new ElevatorControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, elevatorControl);
    }

    private void configMotor() {
        TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.slot0.kP = 0.55;
        VEConfig.slot0.kF = 0.15;
        VEConfig.slot0.kD = 0.03;
        VEConfig.closedloopRamp = 0.2;
        VEConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;
        VEConfig.remoteFilter0.remoteSensorDeviceID = encoderSRX.getDeviceID();
        VEConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
//        VEConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
//        VEConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;

        verticalElevatorMotor.configAllSettings(VEConfig);
        verticalElevatorMotor.brake();

        SparkMaxPIDController HEConfig = horizontalElevatorMotor.getPIDController();
        HEConfig.setP(0.13);
//        HEConfig.setI(0.002);
//        HEConfig.setIZone(200);
//        HEConfig.setD(10);
        HEConfig.setFeedbackDevice(horizontalElevatorMotor.getAlternateEncoder(8192));
        horizontalElevatorMotor.currentLimit(60, 30);
        horizontalElevatorMotor.brake();
    }

    public void setState(Enums.ElevatorState targetState) {
        currentState = targetState;
    }

    public boolean isAtWantedState() {
        return elevatorControl.isAtWantedState();
    }

    public Enums.ElevatorState getTargetState() {
        return currentState;
    }

    public TitanFX getVerticalElevatorMotor() {
        return verticalElevatorMotor;
    }

    public TitanMAX getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }
}