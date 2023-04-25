package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final CANCoder clawOpenCloseEncoder, clawTiltEncoder;
    private final TitanMAX clawTiltNeo;
    private final DigitalInput clawTiltLimitSwitch;

    private final ClawControl clawControl;
    private Enums.ClawState targetState;

    public Claw(TitanSRX clawMainWheelBag,
                TitanSRX clawFollowerWheelBag,
                TitanSRX clawOpenCloseMotor,
                CANCoder clawOpenCloseEncoder,
                TitanMAX clawTiltNeo,
                CANCoder clawTiltEncoder,
                DigitalInput clawTiltLimitSwitch
    ) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawTiltNeo = clawTiltNeo;
        this.clawTiltEncoder = clawTiltEncoder;
        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        this.clawTiltLimitSwitch = clawTiltLimitSwitch;

        configMotor();

        clawControl = new ClawControl(this);
        CommandScheduler.getInstance().setDefaultCommand(this, clawControl);
    }

    private void configMotor() {
        clawMainWheelBag.configFactoryDefault();
        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.follow(clawMainWheelBag);

        clawOpenCloseEncoder.configFactoryDefault();
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.unitString = "deg";
        canCoderConfiguration.sensorDirection = false;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        canCoderConfiguration.magnetOffsetDegrees = -81.387;
        clawOpenCloseEncoder.configAllSettings(canCoderConfiguration);

        clawOpenCloseMotor.configFactoryDefault();
        TalonSRXConfiguration CCConfig = new TalonSRXConfiguration();
        CCConfig.slot0.kP = 2;
        CCConfig.remoteFilter0.remoteSensorDeviceID = clawOpenCloseEncoder.getDeviceID();
        CCConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        CCConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        clawOpenCloseMotor.configAllSettings(CCConfig);
        clawOpenCloseMotor.brake();

        clawTiltNeo.brake();
        clawTiltNeo.currentLimit(25);

        CANCoderConfiguration clawTiltEncoderConfig = new CANCoderConfiguration();
        clawTiltEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        clawTiltEncoderConfig.unitString = "deg";
        clawTiltEncoderConfig.sensorDirection = true;
        clawTiltEncoderConfig.sensorCoefficient = 1.0/4096; // this makes getPosition() return in rotations
        clawTiltEncoderConfig.magnetOffsetDegrees = 57.39;
        clawTiltEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        clawTiltEncoder.configAllSettings(clawTiltEncoderConfig);
    }

    public void setState(Enums.ClawState state) {
        targetState = state;
    }

    public boolean isAtWantedState() {
        return clawControl.isAtWantedState();
    }

    public Enums.ClawState getTargetState() {
        return targetState;
    }

    public TitanSRX getClawWheelMotor() {
        return clawMainWheelBag;
    }

    public TitanSRX getClawFollowerWheelBag() {
        return clawFollowerWheelBag;
    }

    public TitanSRX getClawOpenCloseMotor() {
        return clawOpenCloseMotor;
    }

    public TitanMAX getClawTiltNeo() {
        return clawTiltNeo;
    }

    public CANCoder getClawTiltEncoder() {
        return clawTiltEncoder;
    }

    public DigitalInput getClawTiltLimitSwitch() {
        return clawTiltLimitSwitch;
    }
}