package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
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
    private final ClawControlCommand clawControl;

    public Claw(TitanSRX clawMainWheelBag, TitanSRX clawFollowerWheelBag, TitanSRX clawOpenCloseMotor,
                CANSparkMax clawTiltNeo) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawTiltNeo = clawTiltNeo;
        this.clawOpenCloseMotor = clawOpenCloseMotor;

        configMotor();

        clawControl = new ClawControlCommand(this);
        CommandScheduler.getInstance().setDefaultCommand(this, clawControl);
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
        clawTiltPID.setFeedbackDevice(clawTiltNeo.getAlternateEncoder(8192));
    }

    public void setState(Enums.ClawState state) {
        currentState = state;
        clawControl.setState(currentState);
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
    private final TitanSRX clawWheelMotor, clawOpenCloseMotor;
    private final CANSparkMax clawTiltNeo;

    private double speed = 0, //Claw Intake Wheel Speed
    tiltRotations = 0; //Claw Tilt Rotations
    private int openCloseTicks = 0; //Claw Open Close Ticks

    public ClawControlCommand(Claw claw) {
        this.clawWheelMotor = claw.getClawWheelMotor();
        this.clawOpenCloseMotor = claw.getClawOpenCloseMotor();
        this.clawTiltNeo = claw.getClawTiltNeo();
        addRequirements(claw);
    }

    public void setState(Enums.ClawState state) {
        switch (state) {
            case Claw_RETRACTED:
                speed = 0;
                tiltRotations = 0;
                openCloseTicks = 0;
                break;
            case CLAW_CLOSED:
                speed = 0;
                tiltRotations = 500;
                openCloseTicks = 0;
                break;
            case CLAW_OPEN_SPINNING:
                speed = 1;
                tiltRotations = 500;
                openCloseTicks = 1000;
                break;
            case CLAW_OPEN_STANDBY:
                speed = 0.1;
                tiltRotations = 500;
                openCloseTicks = 1000;
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
                ControlMode.Position,
                openCloseTicks);

        clawTiltNeo.getPIDController().setReference(
                tiltRotations,
                CANSparkMax.ControlType.kPosition);
    }

        @Override
    public boolean isFinished() {
        return false;
    }
//    @Override
//    public boolean isFinished() {
//        final double ticksTolerance = 50;
//        return MathMethods.withinRange(
//                clawTiltNeo.getAlternateEncoder(8196).getPosition(),
//                tiltTicks,
//                ticksTolerance) &&
//                MathMethods.withinRange(
//                        clawOpenCloseMotor.getSelectedSensorPosition(),
//                        openCloseTicks,
//                        ticksTolerance);
//    }
}