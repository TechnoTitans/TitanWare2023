package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;

@SuppressWarnings("unused")
public class Claw extends SubsystemBase {
    private final TitanSRX clawMainWheelBag, clawFollowerWheelBag;
    private final TitanSRX clawOpenCloseMotor;
    private final TitanMAX clawTiltNeo;
    private Enums.ClawState currentState;
    private final ClawControlCommand clawControl;

    public Claw(TitanSRX clawMainWheelBag,
                TitanSRX clawFollowerWheelBag,
                TitanSRX clawOpenCloseMotor,
                TitanMAX clawTiltNeo
    ) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawTiltNeo = clawTiltNeo;
        this.clawOpenCloseMotor = clawOpenCloseMotor;

        this.currentState = Enums.ClawState.CLAW_RETRACTED;

        configMotor();

        clawControl = new ClawControlCommand(this);
        CommandScheduler.getInstance().setDefaultCommand(this, clawControl);
    }

    private void configMotor() {
        clawMainWheelBag.configFactoryDefault();
        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.follow(clawMainWheelBag);

        TalonSRXConfiguration CCConfig = new TalonSRXConfiguration();
        CCConfig.slot0.kP = 0.2; //TODO: TUNE ALL OF THESE
        CCConfig.slot0.kI = 0.002;
        CCConfig.slot0.kD = 10;
        CCConfig.motionAcceleration = 2048 * 5; // 5 rotation per sec
        clawOpenCloseMotor.configAllSettings(CCConfig);

        SparkMaxPIDController clawTiltPID = clawTiltNeo.getPIDController();
        clawTiltPID.setP(0.1);
        clawTiltPID.setI(0.002);
        clawTiltPID.setD(10);
        clawTiltPID.setFF(0.1);
        clawTiltPID.setOutputRange(-0.1, 0.1);
        clawTiltPID.setFeedbackDevice(clawTiltNeo.getAlternateEncoder(8192));
        clawTiltPID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve, 0);
        clawTiltPID.setSmartMotionMaxAccel(25, 0);
        clawTiltPID.setSmartMotionMaxVelocity(50, 0);
        clawTiltPID.setSmartMotionAllowedClosedLoopError(2, 0);
        clawTiltNeo.setClosedLoopRampRate(0.2);
        clawTiltNeo.currentLimit(40, 20);
    }

    public void setState(Enums.ClawState state) {
        currentState = state;
        clawControl.setState(currentState);
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }

    protected TitanSRX getClawWheelMotor() {
        return clawMainWheelBag;
    }

    protected TitanSRX getClawOpenCloseMotor() {
        return clawOpenCloseMotor;
    }

    protected TitanMAX getClawTiltNeo() {
        return clawTiltNeo;
    }
}

@SuppressWarnings("unused")
class ClawControlCommand extends CommandBase {
    private final TitanSRX clawWheelMotor, clawOpenCloseMotor;
    private final TitanMAX clawTiltNeo;

    private ControlMode openCloseControlMode;
    private double speed = 0, //Claw Intake Wheel Speed
    tiltRotations = 0, //Claw Tilt Rotations
    openCloseControl = 0; //Claw Open Close Ticks

    public ClawControlCommand(Claw claw) {
        this.clawWheelMotor = claw.getClawWheelMotor();
        this.clawOpenCloseMotor = claw.getClawOpenCloseMotor();
        this.clawTiltNeo = claw.getClawTiltNeo();
        addRequirements(claw);
    }

    public void setState(Enums.ClawState state) {
        switch (state) {
            case CLAW_RETRACTED:
                speed = 0;
                tiltRotations = 0;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.15;
                break;
            case CLAW_HOLDING:
                speed = 0.15;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.2;
                break;
            case CLAW_OUTTAKE:
                speed = -0.4;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = 0;
                break;
            case CLAW_INTAKING:
                speed = 0.3;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.MotionMagic;
                openCloseControl = -0.1;
                break;
            case CLAW_DROP_CONE:
                speed = 0.2;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = 0.2;
                break;
            case CLAW_STANDBY:
                speed = 0.2;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.MotionMagic;
                openCloseControl = 0.2;
                break;
            default:
                break;
        }
    }

    @Override
    public void execute() {
        clawWheelMotor.set(
                ControlMode.PercentOutput,
                speed);

        clawOpenCloseMotor.set(
                openCloseControlMode,
                openCloseControl);

        clawTiltNeo.set(
                CANSparkMax.ControlType.kSmartMotion,
                tiltRotations);
    }

        @Override
    public boolean isFinished() {
        return false;
    }
}