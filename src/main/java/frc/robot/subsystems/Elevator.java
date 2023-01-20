package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.motors.TitanFX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TitanFX mainVerticalElevatorMotor, followerVerticalElevatorMotor;
    private final CANSparkMax horizontalElevatorMotor;
    private final CANCoder verticalElevatorCanCoder;
    private Enums.ElevatorState currentState;

    public Elevator(TitanFX mainVerticalElevatorMotor, TitanFX followerVerticalElevatorMotor,
                    CANSparkMax horizontalElevatorMotor, CANCoder verticalElevatorCanCoder) {
        this.mainVerticalElevatorMotor = mainVerticalElevatorMotor;
        this.followerVerticalElevatorMotor = followerVerticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.verticalElevatorCanCoder = verticalElevatorCanCoder;

        configMotor();
    }

    private void configMotor() {
        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition; //TUNE DIS
        canCoderConfig.unitString = "deg";
        canCoderConfig.sensorDirection = false;
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.magnetOffsetDegrees = -69;
        verticalElevatorCanCoder.configAllSettings(canCoderConfig);

        TalonFXConfiguration LVEConfig = new TalonFXConfiguration();
        LVEConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        LVEConfig.slot0.kI = 0.002;
        LVEConfig.slot0.integralZone = 200;
        LVEConfig.slot0.kD = 10;
        LVEConfig.slot0.kF = 0.07;
        LVEConfig.closedloopRamp = 0.2;
        LVEConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        LVEConfig.remoteFilter0.remoteSensorDeviceID = verticalElevatorCanCoder.getDeviceID();
        LVEConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        mainVerticalElevatorMotor.configAllSettings(LVEConfig);

        TalonFXConfiguration RVEConfig = new TalonFXConfiguration();
        RVEConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        RVEConfig.slot0.kI = 0.002;
        RVEConfig.slot0.integralZone = 200;
        RVEConfig.slot0.kD = 10;
        RVEConfig.slot0.kF = 0.07;
        RVEConfig.closedloopRamp = 0.2;
        followerVerticalElevatorMotor.configAllSettings(RVEConfig);
        followerVerticalElevatorMotor.follow(mainVerticalElevatorMotor);

        SparkMaxPIDController HVEConfig = horizontalElevatorMotor.getPIDController();
        HVEConfig.setP(0.1);
        HVEConfig.setI(0.002);
        HVEConfig.setIZone(200);
        HVEConfig.setD(10);
        HVEConfig.setFF(0.07);
        HVEConfig.setFeedbackDevice(horizontalElevatorMotor.getAlternateEncoder(8192));
        horizontalElevatorMotor.setClosedLoopRampRate(0.2);
    }

    public void setState(Enums.ElevatorState state) {
        CommandScheduler.getInstance().schedule(new ElevatorControlCommand(this, state));
        currentState = state;
    }

    public Enums.ElevatorState getCurrentState() {
        return currentState;
    }

    protected TitanFX getVerticalElevatorMotor() {
        return mainVerticalElevatorMotor;
    }

    protected CANSparkMax getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }
}

@SuppressWarnings("unused")
class ElevatorControlCommand extends CommandBase {
    private final TitanFX verticalElevatorMotor;
    private final CANSparkMax horizontalElevatorMotor;
    private final Enums.ElevatorState elevatorState;

    double HETargetTicks; //Horizontal Elevator Target Ticks
    double VETargetTicks; //Vertical Elevator Target Ticks

    public ElevatorControlCommand(Elevator elevator, Enums.ElevatorState state) {
        this.elevatorState = state;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
    }

    @Override
    public void initialize() {
        //TODO TUNE THIS
        switch (elevatorState) {
            case ELEVATOR_EXTENDED_HIGH:
                VETargetTicks = 50000;
                HETargetTicks = 50000;
                break;
            case ELEVATOR_EXTENDED_MID:
                VETargetTicks = 4000;
                HETargetTicks = 2500;
                break;
            case ELEVATOR_EXTENDED_GROUND:
                VETargetTicks = 1700;
                HETargetTicks = 1700;
                break;
            case ELEVATOR_STANDBY:
                VETargetTicks = 0;
                HETargetTicks = 1000;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VETargetTicks = 50000;
                HETargetTicks = 2500;
                break;
            case ELEVATOR_PREGAME:
                HETargetTicks = 0;
                VETargetTicks = 0;
                break;
            default:
                return;
        }
        verticalElevatorMotor.set(
                ControlMode.Position,
                VETargetTicks);

        horizontalElevatorMotor.getPIDController().setReference(
                HETargetTicks,
                CANSparkMax.ControlType.kPosition);
    }

    @Override
    public boolean isFinished() {
        final double ticksTolerance = 50;
        return MathMethods.withinRange(
                verticalElevatorMotor.getSelectedSensorPosition(),
                VETargetTicks,
                ticksTolerance) &&
                MathMethods.withinRange(
                        horizontalElevatorMotor.getAlternateEncoder(8196).getPosition(),
                        HETargetTicks,
                        ticksTolerance);
    }
}