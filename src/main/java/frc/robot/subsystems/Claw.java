package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanSRX;

@SuppressWarnings("unused")
public class Claw extends SubsystemBase {
    private final TitanSRX clawMainWheelBag, clawFollowerWheelBag;
    private final TitanSRX clawOpenCloseMotor;
    private final CANSparkMax clawTiltNeo;
    private Enums.ClawState currentState;

    public Claw(TitanSRX clawMainWheelBag, TitanSRX clawFollowerWheelBag, TitanSRX clawOpenCloseMotor,
                CANSparkMax clawTiltNeo) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawTiltNeo = clawTiltNeo;
        this.clawOpenCloseMotor = clawOpenCloseMotor;

        configMotor();
    }

    private void configMotor() {
        TalonSRXConfiguration CWConfig = new TalonSRXConfiguration();
        CWConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        CWConfig.slot0.kI = 0.002;
        CWConfig.slot0.integralZone = 200;
        CWConfig.slot0.kD = 10;
        CWConfig.slot0.kF = 0.1;
        CWConfig.closedloopRamp = 0.2;
        clawMainWheelBag.configAllSettings(CWConfig);
        clawFollowerWheelBag.configAllSettings(CWConfig);
        clawFollowerWheelBag.follow(clawMainWheelBag);

        TalonSRXConfiguration CCConfig = new TalonSRXConfiguration();
        CCConfig.slot0.kP = 0.1; //TODO: TUNE ALL OF THESE
        CCConfig.slot0.kI = 0.002;
        CCConfig.slot0.integralZone = 200;
        CCConfig.slot0.kD = 10;
        CCConfig.neutralDeadband = 0.1;
        CCConfig.closedloopRamp = 0.2;
        clawOpenCloseMotor.configAllSettings(CCConfig);

        SparkMaxPIDController clawTiltPID = clawTiltNeo.getPIDController();
        clawTiltPID.setP(0.1);
        clawTiltPID.setI(0.002);
        clawTiltPID.setD(10);
        clawTiltPID.setFF(0.1);
        clawTiltPID.setFeedbackDevice(clawTiltNeo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle));
    }

    public void setState(Enums.ClawState state) {
        CommandScheduler.getInstance().schedule(new ClawControlCommand(this, state));
        currentState = state;
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }

    protected TitanSRX getClawWheelMotor() {
        return clawFollowerWheelBag;
    }

    protected TitanSRX getClawOpenCloseMotor() {
        return clawOpenCloseMotor;
    }

    protected CANSparkMax getClawTiltNeo() {
        return clawTiltNeo;
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
        final int tiltTicks;
        final int openCloseTicks;

        switch (clawState) {
            case Claw_RETRACTED:
                speed = 0;
                tiltTicks = 0;
                openCloseTicks = 0;
                break;
            case CLAW_CLOSED:
                speed = 0;
                tiltTicks = 500;
                openCloseTicks = 0;
                break;
            case CLAW_OPEN_SPINNING:
                speed = 1;
                tiltTicks = 500;
                openCloseTicks = 1000;
                break;
            case CLAW_OPEN_STANDBY:
                speed = 0.1;
                tiltTicks = 500;
                openCloseTicks = 1000;
                break;
            default:
                return;
        }
        claw.getClawWheelMotor().set(ControlMode.PercentOutput, speed);
        claw.getClawOpenCloseMotor().set(ControlMode.Position, openCloseTicks);
        claw.getClawTiltNeo().getPIDController().setReference(tiltTicks, CANSparkMax.ControlType.kPosition);
    }
}