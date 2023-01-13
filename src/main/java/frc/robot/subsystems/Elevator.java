package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanSRX;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TitanFX leftVerticalElevatorMotor, rightVerticalElevatorMotor;
    private final CANSparkMax horizontalElevatorMotor, clawTiltElevator550;
    private final TitanSRX verticalElevatorSRXMAG, horizontalElevatorSRXMAG, clawTiltElevatorSRXMAG;
    private Enums.ElevatorState currentState;

    public Elevator(TitanFX leftVerticalElevatorMotor, TitanFX rightVerticalElevatorMotor, CANSparkMax clawTiltElevator550,
                    CANSparkMax horizontalElevatorMotor, TitanSRX verticalElevatorSRXMAG, TitanSRX horizontalElevatorSRXMAG,
                    TitanSRX clawTiltElevatorSRXMAG) {
        this.leftVerticalElevatorMotor = leftVerticalElevatorMotor;
        this.rightVerticalElevatorMotor = rightVerticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.clawTiltElevator550 = clawTiltElevator550;
        this.verticalElevatorSRXMAG = verticalElevatorSRXMAG;
        this.horizontalElevatorSRXMAG = horizontalElevatorSRXMAG;
        this.clawTiltElevatorSRXMAG = clawTiltElevatorSRXMAG;

        configMotor();
    }

    private void configMotor() {
        TalonFXConfiguration LVEConfig = new TalonFXConfiguration();
        LVEConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        LVEConfig.slot0.kI = 0.002;
        LVEConfig.slot0.integralZone = 0.002;
        LVEConfig.slot0.kD = 10;
        LVEConfig.closedloopRamp = 0.2;
        LVEConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;
        LVEConfig.remoteFilter0.remoteSensorDeviceID = verticalElevatorSRXMAG.getDeviceID();
        LVEConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        leftVerticalElevatorMotor.configAllSettings(LVEConfig);

        TalonFXConfiguration RVEConfig = new TalonFXConfiguration();
        RVEConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        RVEConfig.slot0.kI = 0.002;
        RVEConfig.slot0.integralZone = 0.002;
        RVEConfig.slot0.kD = 10;
        RVEConfig.closedloopRamp = 0.2;
        rightVerticalElevatorMotor.configAllSettings(RVEConfig);
        rightVerticalElevatorMotor.follow(leftVerticalElevatorMotor);

        SparkMaxPIDController clawTilt550PID = clawTiltElevator550.getPIDController();
        clawTilt550PID.setP(0.1); //TODO: TUNE ALL OF THESE
        clawTilt550PID.setI(0.002);
        clawTilt550PID.setIZone(0.002);
        clawTilt550PID.setD(10);
        clawTiltElevator550.setClosedLoopRampRate(0.2);

        horizontalElevatorMotor.getPIDController().setP(0.1);
        horizontalElevatorMotor.getPIDController().setI(0.002);
        horizontalElevatorMotor.getPIDController().setIZone(200);
        horizontalElevatorMotor.getPIDController().setD(10);
        horizontalElevatorMotor.getPIDController().setFeedbackDevice((MotorFeedbackSensor) horizontalElevatorSRXMAG.getEncoder());
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
        return leftVerticalElevatorMotor;
    }

    protected CANSparkMax getTiltElevatorMotor() {
        return clawTiltElevator550;
    }

    protected CANSparkMax getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }
}

@SuppressWarnings("unused")
class ElevatorControlCommand extends CommandBase {
    private final TitanFX verticalElevatorMotor;
    private final CANSparkMax horizontalElevatorMotor, clawTiltElevator550;
    private final Enums.ElevatorState elevatorState;

    public ElevatorControlCommand(Elevator elevator, Enums.ElevatorState state) {
        this.elevatorState = state;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        this.clawTiltElevator550 = elevator.getTiltElevatorMotor();
    }

    @Override
    public void initialize() {
        //TODO TUNE THIS
        final double keepPositionMotorPercent = 0.07; //7% motor power is always applied to hold elevator position
        //demand0 = encoder ticks motor to go to
        final double HETargetTicks; //Horizontal Elevator Target Ticks
        final double VETargetTicks; //Vertical Elevator Target Ticks
        final double TETargetTicks; //Tilt Claw Target Ticks
        switch (elevatorState) {
            case ELEVATOR_EXTENDED_HIGH:
                VETargetTicks = 50000;
                HETargetTicks = 50000;
                TETargetTicks = 50000;
                break;
            case ELEVATOR_EXTENDED_MID:
                VETargetTicks = 4000;
                HETargetTicks = 2500;
                TETargetTicks = 50000;
                break;
            case ELEVATOR_EXTENDED_GROUND:
                VETargetTicks = 1700;
                HETargetTicks = 1700;
                TETargetTicks = 50000;
                break;
            case ELEVATOR_STANDBY:
                VETargetTicks = 0;
                HETargetTicks = 1000;
                TETargetTicks = 0;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VETargetTicks = 50000;
                HETargetTicks = 2500;
                TETargetTicks = 0;
                break;
            case ELEVATOR_PREGAME:
                HETargetTicks = 0;
                VETargetTicks = 0;
                TETargetTicks = 0;
                break;
            default:
                return;
        }
        verticalElevatorMotor.set(
                ControlMode.Position,
                HETargetTicks,
                DemandType.ArbitraryFeedForward,
                keepPositionMotorPercent);

        horizontalElevatorMotor.getPIDController().setReference(
                VETargetTicks,
                CANSparkMax.ControlType.kPosition,
                0,
                keepPositionMotorPercent);

        clawTiltElevator550.getPIDController().setReference(
                TETargetTicks,
                CANSparkMax.ControlType.kPosition,
                0,
                keepPositionMotorPercent);
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorMotor.stopMotor();
        horizontalElevatorMotor.stopMotor();
        clawTiltElevator550.stopMotor();
    }

}