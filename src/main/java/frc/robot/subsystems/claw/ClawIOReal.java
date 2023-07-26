package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utils.Enums;
import frc.robot.utils.MathUtils;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix5Utils;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class ClawIOReal implements ClawIO {
    private final TalonSRX clawMainWheelBag, clawFollowerWheelBag;
    private final InvertType clawMainWheelBagInverted;
    private final TalonSRX clawOpenCloseMotor;
    private final InvertType clawOpenCloseMotorInverted;
    private final CANCoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanSparkMAX clawTiltNeo;

    private final ProfiledPIDController tiltPID;

    private Enums.ClawState desiredState = Enums.ClawState.CLAW_STANDBY;
    private Enums.ClawState currentState = desiredState;

    private ControlMode openCloseControlMode = currentState.getClawOpenCloseControlMode();
    private Enums.ClawTiltControlMode clawTiltControlMode = currentState.getClawTiltControlMode();

    //Claw Intake Wheel Percent Output
    private double desiredIntakeWheelsPercentOutput = currentState.getIntakeWheelsPercentOutput();
    //Claw Tilt Control Input
    private double desiredTiltControlInput = currentState.getTiltControlInput();
    //Claw Open Close Control Input
    private double desiredOpenCloseControlInput = currentState.getOpenCloseControlInput();

    public ClawIOReal(
            final TalonSRX clawMainWheelBag,
            final TalonSRX clawFollowerWheelBag,
            final InvertType clawMainWheelBagInverted,
            final TalonSRX clawOpenCloseMotor,
            final InvertType clawOpenCloseMotorInverted,
            final CANCoder clawOpenCloseEncoder,
            final TitanSparkMAX clawTiltNeo,
            final CANcoder clawTiltEncoder
    ) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawMainWheelBagInverted = clawMainWheelBagInverted;

        this.clawTiltNeo = clawTiltNeo;
        this.clawTiltEncoder = clawTiltEncoder;

        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        this.clawOpenCloseMotorInverted = clawOpenCloseMotorInverted;

        config();
        setDesiredState(Enums.ClawState.CLAW_STANDBY);

        //TODO: tune pid
        this.tiltPID = new ProfiledPIDController(
                2.7, 0, 0,
                new TrapezoidProfile.Constraints(20, 30)
        );

        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                tiltPID,
                clawTiltEncoder.getAbsolutePosition().waitForUpdate(0.25),
                clawTiltEncoder.getVelocity().waitForUpdate(0.25)
        );
    }

    @Override
    public void periodic() {
        clawMainWheelBag.set(ControlMode.PercentOutput, desiredIntakeWheelsPercentOutput);
        clawOpenCloseMotor.set(
                openCloseControlMode,
                Phoenix5Utils.getPhoenix6To5ControlInput(
                        openCloseControlMode, desiredOpenCloseControlInput
                )
        );

        switch (clawTiltControlMode) {
            case POSITION -> clawTiltNeo.set(tiltPID.calculate(
                    clawTiltEncoder.getAbsolutePosition().refresh().getValue(), desiredTiltControlInput));
            case DUTY_CYCLE -> clawTiltNeo.set(desiredTiltControlInput);
        }
    }

    @Override
    public void updateInputs(final ClawIOInputs inputs) {
        isAtDesiredState();

        inputs.desiredState = desiredState.toString();
        inputs.currentState = currentState.toString();

        inputs.tiltClawControlMode = clawTiltControlMode.toString();
        inputs.currentTiltEncoderPositionRots = clawTiltEncoder.getAbsolutePosition().refresh().getValue();
        inputs.currentTiltEncoderVelocityRotsPerSec = clawTiltEncoder.getVelocity().refresh().getValue();
        inputs.desiredTiltControlInput = desiredTiltControlInput;
        inputs.tiltCurrentAmps = clawTiltNeo.getOutputCurrent();

        inputs.openCloseControlMode = openCloseControlMode.toString();
        inputs.currentOpenCloseEncoderPositionRots = clawOpenCloseEncoder.getAbsolutePosition();
        inputs.currentOpenCloseEncoderVelocityRotsPerSec = clawOpenCloseEncoder.getVelocity();
        inputs.desiredOpenCloseControlInput = desiredOpenCloseControlInput;
        inputs.openCloseCurrentAmps = clawOpenCloseMotor.getStatorCurrent();

        inputs.desiredIntakeWheelsPercentOutput = desiredIntakeWheelsPercentOutput;
    }

    public void config() {
        // Bag Motors
        clawMainWheelBag.configFactoryDefault();
        clawMainWheelBag.setInverted(clawMainWheelBagInverted);

        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.set(ControlMode.Follower, clawMainWheelBag.getDeviceID());
        clawFollowerWheelBag.setInverted(InvertType.OpposeMaster);

        // Claw Open Close Encoder
        final CANCoderConfiguration clawOpenCloseEncoderConfig = new CANCoderConfiguration();
        clawOpenCloseEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        clawOpenCloseEncoderConfig.unitString = Constants.CTRE.PHOENIX_5_CANCODER_UNIT_STRING_ROTS;
        clawOpenCloseEncoderConfig.sensorDirection = false;
        clawOpenCloseEncoderConfig.sensorCoefficient = Constants.CTRE.PHOENIX_5_CANCODER_SENSOR_COEFFICIENT_ROTS;
        clawOpenCloseEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        clawOpenCloseEncoderConfig.magnetOffsetDegrees = -Units.rotationsToDegrees(0.19);

        clawOpenCloseEncoder.configFactoryDefault();
        clawOpenCloseEncoder.configAllSettings(clawOpenCloseEncoderConfig);

        // Claw Open Close Motor
        final TalonSRXConfiguration clawOpenCloseMotorConfig = new TalonSRXConfiguration();
        clawOpenCloseMotorConfig.slot0.kP = 2;
        clawOpenCloseMotorConfig.remoteFilter0.remoteSensorDeviceID = clawOpenCloseEncoder.getDeviceID();
        clawOpenCloseMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        clawOpenCloseMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        clawOpenCloseMotorConfig.continuousCurrentLimit = 10;

        clawOpenCloseMotor.configFactoryDefault();
        clawOpenCloseMotor.configAllSettings(clawOpenCloseMotorConfig);
        clawOpenCloseMotor.setInverted(clawOpenCloseMotorInverted);
        clawOpenCloseMotor.setNeutralMode(NeutralMode.Brake);

        // Claw Tilt Neo
        clawTiltNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        clawTiltNeo.setSmartCurrentLimit(25);

        final CANcoderConfiguration clawTiltEncoderConfig = new CANcoderConfiguration();
        clawTiltEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        clawTiltEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        clawTiltEncoderConfig.MagnetSensor.MagnetOffset = -0.17;

        clawTiltEncoder.getConfigurator().apply(clawTiltEncoderConfig);
    }

    public void setDesiredState(final Enums.ClawState state) {
        desiredState = state;

        desiredIntakeWheelsPercentOutput = state.getIntakeWheelsPercentOutput();
        clawTiltControlMode = state.getClawTiltControlMode();
        desiredTiltControlInput = state.getTiltControlInput();
        openCloseControlMode = state.getClawOpenCloseControlMode();
        desiredOpenCloseControlInput = state.getOpenCloseControlInput();
    }

    // TODO: bring over the isAtDesiredState impl from Sim and test
    public boolean isAtDesiredState() {
        if (currentState == desiredState) {
            return true;
        } else {
            final boolean isAtDesired =
                    MathUtils.withinTolerance(
                            clawOpenCloseEncoder.getAbsolutePosition(),
                            desiredOpenCloseControlInput,
                            0.05
                    ) && MathUtils.withinTolerance(
                            clawTiltEncoder.getAbsolutePosition().refresh().getValue(),
                            desiredTiltControlInput,
                            0.05
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
