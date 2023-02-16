package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.subsystems.ClawControl;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;

@SuppressWarnings("unused")
public class Claw extends SubsystemBase {
    private final TitanSRX clawMainWheelBag, clawFollowerWheelBag;
    private final TitanSRX clawOpenCloseMotor;
    private final CANCoder clawOpenCloseEncoder;
    private final TitanMAX clawTiltNeo;
    private Enums.ClawState currentState;
    private final ColorSensorV3 colorSensor;

    private final ClawControl clawControl;

    public Claw(TitanSRX clawMainWheelBag,
                TitanSRX clawFollowerWheelBag,
                TitanSRX clawOpenCloseMotor,
                CANCoder clawOpenCloseEncoder,
                TitanMAX clawTiltNeo,
                ColorSensorV3 colorSensor
    ) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawTiltNeo = clawTiltNeo;
        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        this.colorSensor = colorSensor;

        clawControl = new ClawControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, clawControl);

        this.currentState = Enums.ClawState.CLAW_HOLDING;

        configMotor();
    }

    private void configMotor() {
        clawMainWheelBag.configFactoryDefault();
        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.follow(clawMainWheelBag);

        TalonSRXConfiguration CCConfig = new TalonSRXConfiguration();
        CCConfig.slot0.kP = 0.2; //TODO: TUNE ALL OF THESE
        CCConfig.slot0.kI = 0.002;
        CCConfig.slot0.kD = 10;
        CCConfig.motionCruiseVelocity = 4096 * 5;
        CCConfig.motionAcceleration = 4096 * 5;
        CCConfig.motionCurveStrength = 3; // S-curve
        CCConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        CCConfig.remoteFilter0.remoteSensorDeviceID = clawOpenCloseEncoder.getDeviceID();
        CCConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        clawOpenCloseMotor.configAllSettings(CCConfig);

        SparkMaxPIDController clawTiltPID = clawTiltNeo.getPIDController();
        clawTiltPID.setP(0.1);
        clawTiltPID.setI(0.002);
        clawTiltPID.setD(10);
        clawTiltPID.setOutputRange(-0.1, 0.1);
        clawTiltPID.setFeedbackDevice(clawTiltNeo.getABSRevBoreThroughEncoder());
        clawTiltPID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve, 0);
        clawTiltPID.setSmartMotionMaxAccel(25, 0);
        clawTiltPID.setSmartMotionMaxVelocity(50, 0);
        clawTiltPID.setSmartMotionAllowedClosedLoopError(2, 0);
        clawTiltNeo.setClosedLoopRampRate(0.2);
        clawTiltNeo.currentLimit(40, 20);
        clawTiltNeo.brake();
    }

    public void setState(Enums.ClawState state) {
        currentState = state;
        clawControl.setState(state);
    }

    public Enums.CurrentGamePiece getCurrentGamePiece() { //TODO: TUNE THIS
        if (colorSensor.getProximity() < 800) {
            return Enums.CurrentGamePiece.NONE;
        } else if (colorSensor.getColor().green > 100) {
            return Enums.CurrentGamePiece.CONE;
        } else if (colorSensor.getProximity() > 800) {
            return Enums.CurrentGamePiece.CUBE;
        } else {
            return Enums.CurrentGamePiece.NONE;
        }
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }

    public TitanSRX getClawWheelMotor() {
        return clawMainWheelBag;
    }

    public TitanSRX getClawOpenCloseMotor() {
        return clawOpenCloseMotor;
    }

    public TitanMAX getClawTiltNeo() {
        return clawTiltNeo;
    }
}