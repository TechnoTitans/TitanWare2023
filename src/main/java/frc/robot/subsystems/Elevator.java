package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenixpro.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.motors.TitanMAX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TalonFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;
    private Enums.ElevatorState currentState;
    private final ElevatorControlCommand elevatorControl;

    public Elevator(TalonFX verticalElevatorMotor,
                    TitanMAX horizontalElevatorMotor) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;

        configMotor();

        this.elevatorControl = new ElevatorControlCommand(this);
        CommandScheduler.getInstance().setDefaultCommand(this, elevatorControl);
    }

    private void configMotor() {
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.2;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 5;

        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitSwitchConfigs.ReverseLimitEnable = true;
        hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        TalonFXConfiguration LVEConfig = new TalonFXConfiguration();
        LVEConfig.Slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        LVEConfig.Slot0.kI = 0.002;
        LVEConfig.Slot0.kD = 10;
        LVEConfig.Slot0.kS = 0.07;
        LVEConfig.ClosedLoopRamps = closedLoopRampsConfigs;
        LVEConfig.MotionMagic = motionMagicConfigs;
        LVEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        LVEConfig.HardwareLimitSwitch = hardwareLimitSwitchConfigs;
        verticalElevatorMotor.getConfigurator().apply(LVEConfig);

        SparkMaxPIDController HVEConfig = horizontalElevatorMotor.getPIDController();
        HVEConfig.setP(0.1);
        HVEConfig.setI(0.002);
        HVEConfig.setIZone(200);
        HVEConfig.setD(10);
        HVEConfig.setFF(0.07);
        HVEConfig.setFeedbackDevice(horizontalElevatorMotor.getABSRevBoreThroughEncoder());
        HVEConfig.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve, 0);
        HVEConfig.setSmartMotionMaxAccel(125, 0);
        HVEConfig.setSmartMotionMaxVelocity(500, 0);
        HVEConfig.setSmartMotionMinOutputVelocity(0, 0);
        HVEConfig.setSmartMotionAllowedClosedLoopError(3, 0);
        horizontalElevatorMotor.setClosedLoopRampRate(0.2);
        horizontalElevatorMotor.currentLimit(60, 30);
    }

    public void setState(Enums.ElevatorState state) {
        currentState = state;
        elevatorControl.setState(state);
    }

    public boolean isAtWantedState() {
        return elevatorControl.isAtWantedState();
    }

    public Enums.ElevatorState getCurrentState() {
        return currentState;
    }

    protected TalonFX getVerticalElevatorMotor() {
        return verticalElevatorMotor;
    }

    protected TitanMAX getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }
}

@SuppressWarnings("unused")
class ElevatorControlCommand extends CommandBase {
    private final TalonFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;

    private double
            HETargetRotations = 0, //Horizontal Elevator Target Ticks
            VEPosition = 0; //Vertical Elevator Target Ticks

    public ElevatorControlCommand(Elevator elevator) {
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        addRequirements(elevator);
    }

    public void setState(Enums.ElevatorState state) {
        //TODO TUNE THIS
        switch (state) {
            case ELEVATOR_EXTENDED_HIGH:
                VEPosition = 50;
                HETargetRotations = 30;
                break;
            case ELEVATOR_EXTENDED_MID:
                VEPosition = 40;
                HETargetRotations = 20;
                break;
            case ELEVATOR_EXTENDED_GROUND:
                VEPosition = 17;
                HETargetRotations = 15;
                break;
            case ELEVATOR_STANDBY:
                VEPosition = 0;
                HETargetRotations = 10;
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

    protected boolean isAtWantedState() {
        return MathMethods.withinRange(
                    verticalElevatorMotor.getPosition().getValue(),
                VEPosition,
                    20) &&
                MathMethods.withinRange(
                    horizontalElevatorMotor.getABSRevBoreThroughEncoder().getPosition(),
                    HETargetRotations,
                    0.1);
    }

    @Override
    public void execute() {
        MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(VEPosition, true, 0, 0, false);
        verticalElevatorMotor.setControl(
                motionMagicDutyCycle);

        horizontalElevatorMotor.set(
                CANSparkMax.ControlType.kSmartMotion,
                HETargetRotations);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}