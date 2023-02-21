package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
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
    private final ColorSensorV3 colorSensor;

    private final ClawControl clawControl;
    private Enums.ClawState targetState = Enums.ClawState.CLAW_STANDBY;

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

        configMotor();

        clawControl = new ClawControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, clawControl);
    }

    private void configMotor() {
        clawMainWheelBag.configFactoryDefault();
        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.follow(clawMainWheelBag);

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.unitString = "deg";
        canCoderConfiguration.magnetOffsetDegrees = -74;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = false;
        clawOpenCloseEncoder.configAllSettings(canCoderConfiguration);

//        TalonSRXConfiguration CCConfig = new TalonSRXConfiguration();
//        CCConfig.slot0.kP = 0.3; //TODO: TUNE ALL OF THESE
//        CCConfig.slot0.kI = 0;
//        CCConfig.slot0.kD = 0;
//        CCConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
//        CCConfig.remoteFilter0.remoteSensorDeviceID = clawOpenCloseEncoder.getDeviceID();
//        CCConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
//        clawOpenCloseMotor.configAllSettings(CCConfig);
        clawOpenCloseMotor.brake();

        SparkMaxPIDController clawTiltPID = clawTiltNeo.getPIDController();
        clawTiltPID.setP(1.2);
        clawTiltPID.setI(0.0);
        clawTiltPID.setD(0);
        clawTiltPID.setOutputRange(-0.5, 0.5);
        clawTiltPID.setFeedbackDevice(clawTiltNeo.getAlternateEncoder(8192));
//        clawTiltNeo.setClosedLoopRampRate(0.2);
//        clawTiltNeo.currentLimit(50, 30);
        clawTiltNeo.brake();
    }

    public void setState(Enums.ClawState state) {
        targetState = state;
    }

    public boolean isAtWantedState() {
        return clawControl.isAtWantedState();
    }

    public Enums.CurrentGamePiece getCurrentGamePiece() { //TODO: TUNE THIS
        if (colorSensor.getProximity() < 800) {
            return Enums.CurrentGamePiece.NONE;
        } else if (colorSensor.getColor().blue > 100) {
            return Enums.CurrentGamePiece.CUBE;
        } else if (colorSensor.getProximity() > 800) {
            return Enums.CurrentGamePiece.CONE;
        } else {
            return Enums.CurrentGamePiece.NONE;
        }
    }

    public Enums.ClawState getTargetState() {
        return targetState;
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

    public double getOpenCloseEncPosition() {
        return clawOpenCloseEncoder.getAbsolutePosition();
    }
}