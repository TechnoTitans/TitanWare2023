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
    private final TitanFX verticalElevatorMotor;
    private ElevatorState currentState;

    public ElevatorSubsystem(TitanFX verticalElevatorMotor) {
        this.verticalElevatorMotor = verticalElevatorMotor;
    }

    private void configMotor(TitanFX VEMotor) {
        TalonFXConfiguration TFXConfig = new TalonFXConfiguration();
        VEMotor.configAllSettings(TFXConfig);
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
}

@SuppressWarnings("unused")
class ElevatorControlCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final ElevatorState elevatorState;

    public ElevatorControlCommand(ElevatorSubsystem elevator, ElevatorState state) {
        this.elevator = elevator;
        this.elevatorState = state;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //TODO TUNE THIS
        final double keepPositionMotorPercent = 0.07; //7% motor power is always applied to hold elevator position
        elevator.getVerticalElevatorMotor().set(ControlMode.MotionMagic, 50, DemandType.ArbitraryFeedForward, keepPositionMotorPercent);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return elevatorState == elevator.getCurrentState();
    }

    @Override
    public void end(boolean interrupted) {
    }

}