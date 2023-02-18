package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.subsystems.ElevatorControl;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanMAX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TalonFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;

    private final ElevatorControl elevatorControl;
    private Enums.ElevatorState currentState;

    public Elevator(TalonFX verticalElevatorMotor,
                    TitanMAX horizontalElevatorMotor) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;

        configMotor();

        this.elevatorControl = new ElevatorControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, elevatorControl);
    }

    private void configMotor() {
        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitSwitchConfigs.ReverseLimitEnable = true;
        hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        VEConfig.Slot0.kI = 0.002;
        VEConfig.Slot0.kD = 10;
        VEConfig.Slot0.kS = 0.07;
        VEConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        VEConfig.MotionMagic.MotionMagicAcceleration = 5;
        VEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEConfig.HardwareLimitSwitch = hardwareLimitSwitchConfigs;
        verticalElevatorMotor.getConfigurator().apply(VEConfig);

        SparkMaxPIDController HEConfig = horizontalElevatorMotor.getPIDController();
        HEConfig.setP(0.1);
        HEConfig.setI(0.002);
        HEConfig.setIZone(200);
        HEConfig.setD(10);
        HEConfig.setFeedbackDevice(horizontalElevatorMotor.getAlternateEncoder(8192));
        horizontalElevatorMotor.currentLimit(60, 30);
        horizontalElevatorMotor.brake();
    }

    public void setState(Enums.ElevatorState state) {
        currentState = state;
    }

    public boolean isAtWantedState() {
        return elevatorControl.isAtWantedState();
    }

    public Enums.ElevatorState getCurrentState() {
        return currentState;
    }

    public TalonFX getVerticalElevatorMotor() {
        return verticalElevatorMotor;
    }

    public TitanMAX getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }
}