package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanMAX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TitanFX mainVerticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;
    private Enums.ElevatorState currentState;
    private final ElevatorControlCommand elevatorControl;

    public Elevator(TitanFX mainVerticalElevatorMotor,
                    TitanMAX horizontalElevatorMotor) {
        this.mainVerticalElevatorMotor = mainVerticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;

        configMotor();

        this.elevatorControl = new ElevatorControlCommand(this);
        CommandScheduler.getInstance().setDefaultCommand(this, elevatorControl);
    }

    private void configMotor() {
        TalonFXConfiguration LVEConfig = new TalonFXConfiguration();
        LVEConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        LVEConfig.slot0.kI = 0.002;
        LVEConfig.slot0.integralZone = 200;
        LVEConfig.slot0.kD = 10;
        LVEConfig.slot0.kF = 0.07;
        LVEConfig.closedloopRamp = 0.2;
        LVEConfig.motionAcceleration = 204.8 * 5; // 5 rotation per sec
        LVEConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        LVEConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        mainVerticalElevatorMotor.configAllSettings(LVEConfig);

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

    protected TitanFX getVerticalElevatorMotor() {
        return mainVerticalElevatorMotor;
    }

    protected TitanMAX getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }
}

@SuppressWarnings("unused")
class ElevatorControlCommand extends CommandBase {
    private final TitanFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;

    private double
            HETargetRotations = 0, //Horizontal Elevator Target Ticks
            VETargetTicks = 0; //Vertical Elevator Target Ticks

    public ElevatorControlCommand(Elevator elevator) {
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        addRequirements(elevator);
    }

    public void setState(Enums.ElevatorState state) {
        //TODO TUNE THIS
        switch (state) {
            case ELEVATOR_EXTENDED_HIGH:
                VETargetTicks = 50000;
                HETargetRotations = 30;
                break;
            case ELEVATOR_EXTENDED_MID:
                VETargetTicks = 4000;
                HETargetRotations = 20;
                break;
            case ELEVATOR_EXTENDED_GROUND:
                VETargetTicks = 1700;
                HETargetRotations = 15;
                break;
            case ELEVATOR_STANDBY:
                VETargetTicks = 0;
                HETargetRotations = 10;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VETargetTicks = 5000;
                HETargetRotations = 15;
                break;
            case ELEVATOR_PREGAME:
                HETargetRotations = 0;
                VETargetTicks = 0;
                break;
            default:
                break;
        }
    }

    protected boolean isAtWantedState() {
        return MathMethods.withinRange(
                    verticalElevatorMotor.getSelectedSensorPosition(),
                    VETargetTicks,
                    20) &&
                MathMethods.withinRange(
                    horizontalElevatorMotor.getABSRevBoreThroughEncoder().getPosition(),
                    HETargetRotations,
                    0.1);
    }

    @Override
    public void execute() {
        verticalElevatorMotor.set(
                ControlMode.MotionMagic,
                VETargetTicks);

        horizontalElevatorMotor.set(
                CANSparkMax.ControlType.kSmartMotion,
                HETargetRotations);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}