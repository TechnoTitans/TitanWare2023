package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;

@SuppressWarnings("unused")
public class Claw extends SubsystemBase {
    private final CANSparkMax clawWheelMotor;
    private final Solenoid clawSolenoid;
    private Enums.ClawState currentState;

    public Claw(CANSparkMax clawWheelMotor, Solenoid clawSolenoid) {
        this.clawWheelMotor = clawWheelMotor;
        this.clawSolenoid = clawSolenoid;

        configMotor();
    }

    private void configMotor() {
        SparkMaxPIDController clawWheelPID = clawWheelMotor.getPIDController();
        clawWheelPID.setP(0.1);
        clawWheelMotor.setSmartCurrentLimit(40);
        clawWheelMotor.setClosedLoopRampRate(0.2);
    }

    public void setState(Enums.ClawState state) {
        CommandScheduler.getInstance().schedule(new ClawControlCommand(this, state));
        currentState = state;
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }

    protected CANSparkMax getClawWheelMotor() {
        return clawWheelMotor;
    }

    protected Solenoid getClawSolenoid() {
        return clawSolenoid;
    }
}

@SuppressWarnings("unused")
class ClawControlCommand extends CommandBase {
    private final Claw claw;
    private final Enums.ClawState clawState;

    public ClawControlCommand(Claw claw, Enums.ClawState state) {
        this.claw = claw;
        this.clawState = state;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        final double speed;
        final boolean clawOpen;
        switch (clawState) {
            case CLAW_CLOSED:
                speed = 0;
                clawOpen = false;
                break;
            case CLAW_OPEN_SPINNING:
                speed = 1;
                clawOpen = true;
                break;
            case CLAW_OPEN_STOPPED:
                speed = 0;
                clawOpen = true;
                break;
            default:
                return;
        }
        claw.getClawSolenoid().set(clawOpen);
        claw.getClawWheelMotor().set(speed);
    }
}