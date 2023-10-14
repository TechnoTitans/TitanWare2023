package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.SimConstants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerInverted;
    private final SensorDirectionValue verticalElevatorEncoderR;
    private final TitanSparkMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorRearLimitSwitch;

    private SuperstructureStates.ElevatorState desiredState = SuperstructureStates.ElevatorState.ELEVATOR_RESET;
    private final ProfiledPIDController horizontalElevatorPID;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;

    // Cached StatusSignals
    private final StatusSignal<Double> _verticalPosition;
    private final StatusSignal<Double> _verticalVelocity;
    private final StatusSignal<Double> _verticalDutyCycle;
    private final StatusSignal<Double> _verticalMotorTorqueCurrent;
    private final StatusSignal<Double> _verticalMotorFollowerTorqueCurrent;
    private final StatusSignal<Double> _verticalMotorDeviceTemp;
    private final StatusSignal<Double> _verticalMotorFollowerDeviceTemp;
    private final StatusSignal<Double> _horizontalPosition;
    private final StatusSignal<Double> _horizontalVelocity;

    private SuperstructureStates.VerticalElevatorMode verticalElevatorMode =
            desiredState.getVerticalElevatorMode();

    /**
     * Horizontal Elevator Control Input
     * <p>Units can be PositionRots, DutyCycle</p>
     */
    private double VEControlInput = desiredState.getVEControlInput();
    private SuperstructureStates.HorizontalElevatorMode horizontalElevatorMode =
            desiredState.getHorizontalElevatorMode();

    /**
     * Vertical Elevator Control Input
     * <p>Units can be PositionRots, DutyCycle</p>
     * @see SuperstructureStates.VerticalElevatorMode
     */
    private double HEControlInput = desiredState.getHEControlInput();

    private boolean verticalElevatorReset = false;
    private boolean horizontalElevatorReset = false;
    private boolean elevatorsHaveReset = false;

    public ElevatorIOReal(
            final TalonFX verticalElevatorMotor,
            final InvertedValue verticalElevatorMotorR,
            final TalonFX verticalElevatorMotorFollower,
            final InvertedValue verticalElevatorMotorFollowerInverted,
            final CANcoder verticalElevatorEncoder,
            final CANcoder horizontalElevatorEncoder,
            final SensorDirectionValue verticalElevatorEncoderR,
            final TitanSparkMAX horizontalElevatorMotor,
            final DigitalInput verticalElevatorLimitSwitch,
            final DigitalInput horizontalElevatorRearLimitSwitch
    ) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorR = verticalElevatorMotorR;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerInverted = verticalElevatorMotorFollowerInverted;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.verticalElevatorEncoderR = verticalElevatorEncoderR;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;
        this.horizontalElevatorRearLimitSwitch = horizontalElevatorRearLimitSwitch;

        this.horizontalElevatorPID = new ProfiledPIDController(
                0.5, 0, 0,
                new TrapezoidProfile.Constraints(10, 13)
        );

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicVoltage = new MotionMagicVoltage(0);
        this.dutyCycleOut = new DutyCycleOut(0);

        this._verticalPosition = verticalElevatorMotor.getPosition();
        this._verticalVelocity = verticalElevatorMotor.getVelocity();
        this._verticalDutyCycle = verticalElevatorMotor.getDutyCycle();
        this._verticalMotorTorqueCurrent = verticalElevatorMotor.getTorqueCurrent();
        this._verticalMotorFollowerTorqueCurrent = verticalElevatorMotorFollower.getTorqueCurrent();
        this._verticalMotorDeviceTemp = verticalElevatorMotor.getDeviceTemp();
        this._verticalMotorFollowerDeviceTemp = verticalElevatorMotorFollower.getDeviceTemp();
        this._horizontalPosition = horizontalElevatorEncoder.getPosition();
        this._horizontalVelocity = horizontalElevatorEncoder.getVelocity();
    }

    @Override
    public void initialize() {
        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                horizontalElevatorPID,
                _horizontalPosition.waitForUpdate(0.25),
                _horizontalVelocity.waitForUpdate(0.25)
        );
    }

    private boolean resetElevator() {
        if (!verticalElevatorReset && verticalElevatorLimitSwitch.get()) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = SuperstructureStates.VerticalElevatorMode.DUTY_CYCLE;
            VEControlInput = 0;
            verticalElevatorReset = true;
        }

        if (!horizontalElevatorReset && horizontalElevatorRearLimitSwitch.get()) {
            horizontalElevatorEncoder.setPosition(0);
            horizontalElevatorMode = SuperstructureStates.HorizontalElevatorMode.DUTY_CYCLE;
            HEControlInput = 0;
            horizontalElevatorReset = true;

            PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                    horizontalElevatorPID,
                    _horizontalPosition.waitForUpdate(0.25),
                    _horizontalVelocity.waitForUpdate(0.25)
            );
        }

        return verticalElevatorReset && horizontalElevatorReset;
    }

    private double getVEPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_verticalPosition, _verticalVelocity);
    }

    private double getHEPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_horizontalPosition, _horizontalVelocity);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic() {
        if (desiredState == SuperstructureStates.ElevatorState.ELEVATOR_RESET && !elevatorsHaveReset) {
            elevatorsHaveReset = resetElevator();
            if (elevatorsHaveReset) {
                setDesiredState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY);
            }

            Logger.getInstance().recordOutput("ElevatorsResetSuccess", elevatorsHaveReset);
        }

        switch (verticalElevatorMode) {
            case POSITION -> verticalElevatorMotor.setControl(
                    positionVoltage.withPosition(VEControlInput)
            );
            case MOTION_MAGIC -> verticalElevatorMotor.setControl(
                    motionMagicVoltage.withPosition(VEControlInput)
            );
            case DUTY_CYCLE -> verticalElevatorMotor.setControl(
                    dutyCycleOut.withOutput(VEControlInput)
            );
        }

        switch (horizontalElevatorMode) {
            case POSITION -> horizontalElevatorMotor.getPIDController().setReference(
                    horizontalElevatorPID.calculate(
                            _horizontalPosition.refresh().getValue(),
                            HEControlInput
                    ),
                    CANSparkMax.ControlType.kDutyCycle
            );
            case DUTY_CYCLE -> horizontalElevatorMotor.getPIDController().setReference(
                    HEControlInput,
                    CANSparkMax.ControlType.kDutyCycle
            );
        }
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        inputs.verticalEncoderPositionRots = getVEPosition();
        inputs.verticalEncoderVelocityRotsPerSec = _verticalVelocity.refresh().getValue();
        inputs.verticalMotorDutyCycle = _verticalDutyCycle.refresh().getValue();
        inputs.verticalMotorCurrentsAmps = new double[] {
                _verticalMotorTorqueCurrent.refresh().getValue(),
                _verticalMotorFollowerTorqueCurrent.refresh().getValue()
        };
        inputs.verticalMotorTempsCelsius = new double[] {
                _verticalMotorDeviceTemp.refresh().getValue(),
                _verticalMotorFollowerDeviceTemp.refresh().getValue()
        };

        inputs.horizontalEncoderPositionRots = getHEPosition();
        inputs.horizontalEncoderVelocityRotsPerSec = _horizontalVelocity.refresh().getValue();
        inputs.horizontalMotorDutyCycle = horizontalElevatorMotor.getAppliedOutput();
        inputs.horizontalMotorTempCelsius = horizontalElevatorMotor.getMotorTemperature();

        inputs.verticalLimitSwitch = verticalElevatorLimitSwitch.get();
        inputs.horizontalLimitSwitch = horizontalElevatorRearLimitSwitch.get();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        // Vertical elevator CANCoder
        final CANcoderConfiguration verticalElevatorEncoderConfig = new CANcoderConfiguration();
        verticalElevatorEncoderConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderR;

        verticalElevatorEncoder.getConfigurator().apply(verticalElevatorEncoderConfig);

        // Vertical elevator motor
        final TalonFXConfiguration verticalElevatorMotorConfig = new TalonFXConfiguration();
        verticalElevatorMotorConfig.Slot0 = new Slot0Configs(22, 0, 0, 0);
        verticalElevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        verticalElevatorMotorConfig.Feedback.RotorToSensorRatio = SimConstants.Elevator.Vertical.GEARING;
        verticalElevatorMotorConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();
        verticalElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        verticalElevatorMotorConfig.MotorOutput.Inverted = verticalElevatorMotorR;
        verticalElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        verticalElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        // TODO: tune motion magic config
        verticalElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
        verticalElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 40;
        verticalElevatorMotorConfig.MotionMagic.MotionMagicJerk = 100;

        verticalElevatorMotor.getConfigurator().apply(verticalElevatorMotorConfig);

        // Vertical elevator motor follower
        final TalonFXConfiguration verticalElevatorMotorFollowerConfig = new TalonFXConfiguration();
        verticalElevatorMotorFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        verticalElevatorMotorFollowerConfig.MotorOutput.Inverted = verticalElevatorMotorFollowerInverted;
        verticalElevatorMotorFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        verticalElevatorMotorFollowerConfig.CurrentLimits.StatorCurrentLimit = 60;

        verticalElevatorMotorFollower.getConfigurator().apply(verticalElevatorMotorFollowerConfig);
        verticalElevatorMotorFollower.setControl(new Follower(
                verticalElevatorMotor.getDeviceID(),
                false
        ));

        // Horizontal elevator motor
        horizontalElevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Horizontal elevator CANCoder
        final CANcoderConfiguration horizontalElevatorEncoderConfig = new CANcoderConfiguration();
        horizontalElevatorEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        horizontalElevatorEncoder.getConfigurator().apply(horizontalElevatorEncoderConfig);
    }

    @Override
    public void setDesiredState(final SuperstructureStates.ElevatorState desiredState) {
        this.desiredState = desiredState;
        this.verticalElevatorMode = desiredState.getVerticalElevatorMode();
        this.VEControlInput = desiredState.getVEControlInput();
        this.horizontalElevatorMode = desiredState.getHorizontalElevatorMode();
        this.HEControlInput = desiredState.getHEControlInput();
    }
}
