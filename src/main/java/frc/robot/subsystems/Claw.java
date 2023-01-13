package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanSRX;

@SuppressWarnings("unused")
public class Claw extends SubsystemBase {
    private final TitanSRX clawLeftWheelBag, clawRightWheelBag;
    private final Solenoid clawSolenoid;
    private Enums.ClawState currentState;

    public Claw(TitanSRX clawLeftWheelBag, TitanSRX clawRightWheelBag, Solenoid clawSolenoid) {
        this.clawLeftWheelBag = clawLeftWheelBag;
        this.clawRightWheelBag = clawRightWheelBag;
        this.clawSolenoid = clawSolenoid;

        configMotor();
    }

    private void configMotor() {
        TalonSRXConfiguration CWConfig = new TalonSRXConfiguration();
        CWConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        CWConfig.slot0.kI = 0.002;
        CWConfig.slot0.integralZone = 0.002;
        CWConfig.slot0.kD = 10;
        CWConfig.slot0.kF = 0.1;
        CWConfig.closedloopRamp = 0.2;
        clawLeftWheelBag.configAllSettings(CWConfig);
        clawRightWheelBag.configAllSettings(CWConfig);
        clawRightWheelBag.follow(clawLeftWheelBag);
    }

    public void setState(Enums.ClawState state) {
        CommandScheduler.getInstance().schedule(new ClawControlCommand(this, state));
        currentState = state;
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }

    protected TitanSRX getClawWheelMotor() {
        return clawRightWheelBag;
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
            case CLAW_OPEN_STANDBY:
                speed = 0.1;
                clawOpen = true;
                break;
            default:
                return;
        }
        claw.getClawSolenoid().set(clawOpen);
        claw.getClawWheelMotor().set(ControlMode.PercentOutput, speed);
    }
}