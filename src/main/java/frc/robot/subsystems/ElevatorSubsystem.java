package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.motors.TitanFX;

@SuppressWarnings("unused")
enum ElevatorState {
    ELEVATOR_EXTENDED_HIGH, //Elevator High and Dropper extended
    ELEVATOR_EXTENDED_MID, //Elevator Mid and Dropper extended
    ELEVATOR_EXTENDED_LOW, //Elevator Low and Dropper extended
    ELEVATOR_STANDBY, //Elevator at pickup height and grabber open
    ELEVATOR_PREGAME //Elevator at pickup height and grabber tilted vertically
}

@SuppressWarnings("unused")
public class ElevatorSubsystem extends SubsystemBase {
    private final TitanFX verticalElevatorMotor, horizontalElevatorMotor, tiltElevatorMotor;
    private ElevatorState currentState;

    public ElevatorSubsystem(TitanFX verticalElevatorMotor, TitanFX horizontalElevatorMotor, TitanFX tiltElevatorMotor) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.tiltElevatorMotor = tiltElevatorMotor;

        configMotor(verticalElevatorMotor, horizontalElevatorMotor, tiltElevatorMotor);
    }

    private void configMotor(TitanFX VEMotor, TitanFX HEMotor, TitanFX TEMotor) {
        TalonFXConfiguration VETFXConfig = new TalonFXConfiguration();
        VETFXConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        VETFXConfig.slot0.kI = 0.002;
        VETFXConfig.slot0.kD = 10;
        VEMotor.configAllSettings(VETFXConfig);

        TalonFXConfiguration HETFXConfig = new TalonFXConfiguration();
        HETFXConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        HETFXConfig.slot0.kI = 0.002;
        HETFXConfig.slot0.kD = 10;
        HEMotor.configAllSettings(HETFXConfig);

        TalonFXConfiguration TETFXConfig = new TalonFXConfiguration();
        TETFXConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        TETFXConfig.slot0.kI = 0.002;
        TETFXConfig.slot0.kD = 10;
        TEMotor.configAllSettings(TETFXConfig);
    }

    public void setState(ElevatorState state) {
        CommandScheduler.getInstance().schedule(new ElevatorControlCommand(this, state));
        currentState = state;
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    protected TitanFX getVerticalElevatorMotor() {
        return verticalElevatorMotor;
    }
    protected TitanFX getHorizontalElevatorMotor() {
        return horizontalElevatorMotor;
    }

        protected TitanFX getTiltElevatorMotor() {
        return tiltElevatorMotor;
    }
}

@SuppressWarnings("unused")
class ElevatorControlCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final TitanFX verticalElevatorMotor, horizontalElevatorMotor, tiltElevatorMotor;
    private final ElevatorState elevatorState;

    public ElevatorControlCommand(ElevatorSubsystem elevator, ElevatorState state) {
        this.elevator = elevator;
        this.elevatorState = state;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        this.tiltElevatorMotor = elevator.getTiltElevatorMotor();
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //TODO TUNE THIS
        final double keepPositionMotorPercent = 0.07; //7% motor power is always applied to hold elevator position
        //demand0 = encoder ticks to be at
        final double HETargetTicks;
        final double VETargetTicks;
        final double TETargetTicks;
        switch (elevatorState) {
            case ELEVATOR_EXTENDED_HIGH:
                HETargetTicks = 50000;
                VETargetTicks = 50000;
                TETargetTicks = 50000;
                break;
            case ELEVATOR_EXTENDED_MID:
                HETargetTicks = 2500;
                VETargetTicks = 2500;
                TETargetTicks = 50000;
                break;
            case ELEVATOR_EXTENDED_LOW:
                HETargetTicks = 1700;
                VETargetTicks = 1700;
                TETargetTicks = 50000;
                break;
            case ELEVATOR_STANDBY:
                HETargetTicks = 1000;
                VETargetTicks = 0;
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
        horizontalElevatorMotor.set(ControlMode.Position, VETargetTicks, DemandType.ArbitraryFeedForward, keepPositionMotorPercent);
        verticalElevatorMotor.set(ControlMode.Position, HETargetTicks, DemandType.ArbitraryFeedForward, keepPositionMotorPercent);
        tiltElevatorMotor.set(ControlMode.Position, TETargetTicks, DemandType.ArbitraryFeedForward, keepPositionMotorPercent);
    }

//    @Override
//    public void execute() {
//
//    }

    @Override
    public boolean isFinished() {
        return elevatorState == elevator.getCurrentState();
    }

    @Override
    public void end(boolean interrupted) {
    }

}