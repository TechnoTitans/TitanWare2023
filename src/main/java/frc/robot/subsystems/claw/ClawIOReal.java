package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix5Utils;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class ClawIOReal implements ClawIO {
    private final TalonSRX clawMainWheelBag, clawFollowerWheelBag;
    private final TalonSRX clawOpenCloseMotor;
    private final CANcoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanSparkMAX clawTiltNeo;

    private final ArmFeedforward armFeedforward;
    private final ProfiledPIDController tiltPID;

    private final PIDController openClosePID;

    // Cached StatusSignals
    private final StatusSignal<Double> _tiltPosition;
    private final StatusSignal<Double> _tiltVelocity;
    private final StatusSignal<Double> _openClosePosition;
    private final StatusSignal<Double> _openCloseVelocity;

    private SuperstructureStates.ClawOpenCloseControlMode openCloseControlMode;
    private SuperstructureStates.ClawTiltControlMode clawTiltControlMode;

    //Claw Intake Wheel Percent Output
    private double desiredIntakeWheelsPercentOutput;
    //Claw Tilt Control Input
    private double desiredTiltControlInput;
    //Claw Open Close Control Input
    private double desiredOpenCloseControlInput;

    public ClawIOReal(final HardwareConstants.ClawConstants clawConstants) {
        this.clawMainWheelBag = new TalonSRX(clawConstants.clawMainWheelMotorId());
        this.clawFollowerWheelBag = new TalonSRX(clawConstants.clawFollowerWheelMotorId());

        this.clawOpenCloseMotor = new TalonSRX(clawConstants.clawOpenCloseMotorId());
        this.clawOpenCloseEncoder = new CANcoder(clawConstants.clawOpenCloseEncoderId(), clawConstants.clawCANBus());

        this.clawTiltNeo = new TitanSparkMAX(clawConstants.clawTiltMotorId(), CANSparkMaxLowLevel.MotorType.kBrushless);
        this.clawTiltEncoder = new CANcoder(clawConstants.clawTiltEncoderId(), clawConstants.clawCANBus());

        //TODO: tune pid in real
        this.armFeedforward = new ArmFeedforward(0, 0.05, 0, 0);
        this.tiltPID = new ProfiledPIDController(
                3, 0, 0,
                new TrapezoidProfile.Constraints(8, 12)
        );

        this.openClosePID = new PIDController(
                2, 0, 0
        );

        this._tiltPosition = clawTiltEncoder.getAbsolutePosition();
        this._tiltVelocity = clawTiltEncoder.getVelocity();
        this._openClosePosition = clawOpenCloseEncoder.getAbsolutePosition();
        this._openCloseVelocity = clawOpenCloseEncoder.getVelocity();
    }

    @Override
    public void initialize() {
        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                tiltPID,
                _tiltPosition.waitForUpdate(0.25),
                _tiltVelocity.waitForUpdate(0.25)
        );
    }

    @Override
    public void periodic() {
        clawMainWheelBag.set(ControlMode.PercentOutput, desiredIntakeWheelsPercentOutput);

        switch (openCloseControlMode) {
            case POSITION -> clawOpenCloseMotor.set(
                    ControlMode.PercentOutput,
                    openClosePID.calculate(
                            _openClosePosition.refresh().getValue(),
                            desiredOpenCloseControlInput
                    )
            );
            case DUTY_CYCLE -> clawOpenCloseMotor.set(
                    ControlMode.PercentOutput,
                    Phoenix5Utils.getPhoenix6To5ControlInput(
                            openCloseControlMode.getControlMode(), desiredOpenCloseControlInput
                    )
            );
        }

        switch (clawTiltControlMode) {
            case POSITION -> clawTiltNeo.getPIDController().setReference(
                    tiltPID.calculate(
                            _tiltPosition.refresh().getValue(),
                            desiredTiltControlInput
                    ) + armFeedforward.calculate(
                            Units.rotationsToRadians(desiredTiltControlInput - 0.315),
                            0
                    ),
                    CANSparkMax.ControlType.kDutyCycle
            );
            case DUTY_CYCLE -> clawTiltNeo.getPIDController().setReference(
                    desiredTiltControlInput,
                    CANSparkMax.ControlType.kDutyCycle
            );
        }
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ClawIOInputs inputs) {
        inputs.tiltEncoderPositionRots = _tiltPosition.refresh().getValue();
        inputs.tiltEncoderVelocityRotsPerSec = _tiltVelocity.refresh().getValue();
        inputs.tiltPercentOutput = clawTiltNeo.getAppliedOutput();
        inputs.tiltCurrentAmps = clawTiltNeo.getOutputCurrent();
        inputs.tiltTempCelsius = clawTiltNeo.getMotorTemperature();

        inputs.openCloseEncoderPositionRots = _openClosePosition.refresh().getValue();
        inputs.openCloseEncoderVelocityRotsPerSec = _openCloseVelocity.refresh().getValue();
        inputs.openClosePercentOutput = clawOpenCloseMotor.getMotorOutputPercent();
        inputs.openCloseCurrentAmps = clawOpenCloseMotor.getStatorCurrent();
        inputs.openCloseMotorControllerTempCelsius = clawOpenCloseMotor.getTemperature();

        inputs.intakeWheelsPercentOutput = clawMainWheelBag.getMotorOutputPercent();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        // Bag Motors
        clawMainWheelBag.configFactoryDefault();
        clawMainWheelBag.setInverted(InvertType.None);

        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.set(ControlMode.Follower, clawMainWheelBag.getDeviceID());
        clawFollowerWheelBag.setInverted(InvertType.OpposeMaster);

        // Claw Open Close Encoder
        final CANcoderConfiguration clawOpenCloseEncoderConfig = new CANcoderConfiguration();
        clawOpenCloseEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        clawOpenCloseEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        clawOpenCloseEncoderConfig.MagnetSensor.MagnetOffset = 0.28;

        clawOpenCloseEncoder.getConfigurator().apply(clawOpenCloseEncoderConfig);

        // Claw Open Close Motor
        final TalonSRXConfiguration clawOpenCloseMotorConfig = new TalonSRXConfiguration();
        clawOpenCloseMotorConfig.continuousCurrentLimit = 10;

        clawOpenCloseMotor.configFactoryDefault();
        clawOpenCloseMotor.configAllSettings(clawOpenCloseMotorConfig);
        clawOpenCloseMotor.setInverted(InvertType.None);
        clawOpenCloseMotor.setNeutralMode(NeutralMode.Brake);

        // Claw Tilt Neo
        clawTiltNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        clawTiltNeo.setSmartCurrentLimit(25);

        final CANcoderConfiguration clawTiltEncoderConfig = new CANcoderConfiguration();
        clawTiltEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        clawTiltEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        clawTiltEncoderConfig.MagnetSensor.MagnetOffset = 0.48;

        clawTiltEncoder.getConfigurator().apply(clawTiltEncoderConfig);
    }

    @Override
    public void setDesiredState(final SuperstructureStates.ClawState desiredState) {
        this.desiredIntakeWheelsPercentOutput = desiredState.getIntakeWheelsPercentOutput();
        this.clawTiltControlMode = desiredState.getClawTiltControlMode();
        this.desiredTiltControlInput = desiredState.getTiltControlInput();
        this.openCloseControlMode = desiredState.getClawOpenCloseControlMode();
        this.desiredOpenCloseControlInput = desiredState.getOpenCloseControlInput();
    }
}
