package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utils.Enums;
import frc.robot.utils.MathUtils;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;

import java.time.Clock;

public class ClawIOReal implements ClawIO {
    private final TitanSRX clawMainWheelBag, clawFollowerWheelBag;
    private final TitanSRX clawOpenCloseMotor;
    private final CANCoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanMAX clawTiltNeo;
    private final DigitalInput clawTiltLimitSwitch;

    private final ProfiledPIDController tiltPID;

    private Enums.ClawState desiredState;
    private Enums.ClawState currentState;

    private ControlMode openCloseControlMode;
    private Enums.ClawControlMode clawControlMode;

    //Claw Intake Wheel Speed
    private double desiredIntakeWheelsPercentOutput = 0;
    //Claw Tilt Rotations
    private double desiredTiltPositionTicks = 0;
    //Claw Open Close Ticks
    private double desiredOpenClosePositionTicks = 0;

    public ClawIOReal(
            final TitanSRX clawMainWheelBag,
            final TitanSRX clawFollowerWheelBag,
            final TitanSRX clawOpenCloseMotor,
            final CANCoder clawOpenCloseEncoder,
            final TitanMAX clawTiltNeo,
            final CANcoder clawTiltEncoder,
            final DigitalInput clawTiltLimitSwitch
    ) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawTiltNeo = clawTiltNeo;
        this.clawTiltEncoder = clawTiltEncoder;
        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        this.clawTiltLimitSwitch = clawTiltLimitSwitch;

        //TODO: TOOK THIS
        this.tiltPID = new ProfiledPIDController(
                3, 0, 0,
                new TrapezoidProfile.Constraints(3, 5)
        );

        config();
        setDesiredState(Enums.ClawState.CLAW_STANDBY);
    }

    @Override
    public void periodic() {
        if (clawTiltLimitSwitch.get() && clawControlMode == Enums.ClawControlMode.DUTY_CYCLE) {
            clawTiltEncoder.setPosition(0);
            desiredTiltPositionTicks = 0.1;
        }

        clawMainWheelBag.set(
                ControlMode.PercentOutput,
                desiredIntakeWheelsPercentOutput);

        clawOpenCloseMotor.set(
                openCloseControlMode,
                desiredOpenClosePositionTicks);

        switch (clawControlMode) {
            case POSITION -> clawTiltNeo.set(tiltPID.calculate(
                    clawTiltEncoder.getAbsolutePosition().refresh().getValue(), desiredTiltPositionTicks));
            case DUTY_CYCLE -> clawTiltNeo.set(desiredTiltPositionTicks);
        }
    }

    @Override
    public void updateInputs(final ClawIOInputs inputs) {
        isAtDesiredState();

        inputs.currentTiltEncoderPositionTicks = clawTiltEncoder.getAbsolutePosition().refresh().getValue();
        inputs.desiredTiltPositionTicks = desiredTiltPositionTicks;
        inputs.currentOpenCloseEncoderPositionTicks = clawOpenCloseEncoder.getAbsolutePosition();
        inputs.desiredOpenClosePositionTicks = desiredOpenClosePositionTicks;
        inputs.desiredIntakeWheelsPercentOutput = desiredIntakeWheelsPercentOutput;
        inputs.openCloseCurrentAmps = clawOpenCloseMotor.getCurrent();
        inputs.openCloseControlMode = openCloseControlMode.toString();
        inputs.tiltClawControlMode = clawControlMode.toString();
        inputs.desiredState = desiredState.toString();
        inputs.currentState = currentState.toString();
        inputs.tiltLimitSwitch = clawTiltLimitSwitch.get();
    }

    public void config() {
        clawMainWheelBag.configFactoryDefault();
        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.follow(clawMainWheelBag);

        final CANCoderConfiguration clawOpenCloseEncoderConfig = new CANCoderConfiguration();
        clawOpenCloseEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        clawOpenCloseEncoderConfig.unitString = Constants.CTRE.PHOENIX_5_CANCODER_UNIT_STRING_ROTS;
        clawOpenCloseEncoderConfig.sensorDirection = false;
        clawOpenCloseEncoderConfig.sensorCoefficient = Constants.CTRE.PHOENIX_5_CANCODER_SENSOR_COEFFICIENT_ROTS;
        clawOpenCloseEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        clawOpenCloseEncoderConfig.magnetOffsetDegrees = -81.387;

        clawOpenCloseEncoder.configFactoryDefault();
        clawOpenCloseEncoder.configAllSettings(clawOpenCloseEncoderConfig);

        final TalonSRXConfiguration CCConfig = new TalonSRXConfiguration();
        CCConfig.slot0.kP = 2;
        CCConfig.remoteFilter0.remoteSensorDeviceID = clawOpenCloseEncoder.getDeviceID();
        CCConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        CCConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        CCConfig.continuousCurrentLimit = 10;

        clawOpenCloseMotor.configFactoryDefault();
        clawOpenCloseMotor.configAllSettings(CCConfig);
        clawOpenCloseMotor.brake();

        clawTiltNeo.brake();
        clawTiltNeo.currentLimit(25);

        final CANcoderConfiguration clawTiltEncoderConfig = new CANcoderConfiguration();
        clawTiltEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        clawTiltEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        clawTiltEncoderConfig.MagnetSensor.MagnetOffset = -0.17;

        clawTiltEncoder.getConfigurator().apply(clawTiltEncoderConfig);
    }

    public void setDesiredState(final Enums.ClawState state) {
        desiredState = state;

        desiredIntakeWheelsPercentOutput = state.getIntakeWheelsPercentOutput();
        clawControlMode = state.getClawControlMode();
        desiredTiltPositionTicks = state.getTiltPositionRots();
        openCloseControlMode = state.getOpenCloseControlMode();
        desiredOpenClosePositionTicks = state.getOpenCloseRots();
    }

    public boolean isAtDesiredState() {
        if (currentState == desiredState) {
            return true;
        } else {
            final boolean isAtDesired =
                    MathUtils.withinRange(
                            clawOpenCloseEncoder.getAbsolutePosition(),
                            desiredOpenClosePositionTicks,
                            5
                    ) && MathUtils.withinRange(
                            clawTiltEncoder.getAbsolutePosition().refresh().getValue(),
                            desiredTiltPositionTicks,
                            5
                    );

            if (isAtDesired) {
                currentState = desiredState;
            }

            return isAtDesired;
        }
    }

    public Enums.ClawState getDesiredState() {
        return desiredState;
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }
}
