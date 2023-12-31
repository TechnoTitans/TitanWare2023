package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.ctre.Phoenix6Utils;

public class ElevatorIOFalcon implements ElevatorIO {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final TalonFX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorRearLimitSwitch;

    private SuperstructureStates.ElevatorState desiredState = SuperstructureStates.ElevatorState.ELEVATOR_RESET;
    private SuperstructureStates.ElevatorState lastDesiredState = desiredState;

    private final PositionVoltage verticalPositionVoltage;
    private final DynamicMotionMagicVoltage verticalMotionMagicVoltage;
    private final DutyCycleOut verticalDutyCycleOut;

    private final DutyCycleOut horizontalDutyCycleOut;
    private final MotionMagicVoltage horizontalMotionMagicVoltage;

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

    public ElevatorIOFalcon(final HardwareConstants.ElevatorConstants elevatorConstants) {
        this.verticalElevatorMotor = new TalonFX(
                elevatorConstants.verticalMainMotorId(),
                elevatorConstants.verticalElevatorCANBus()
        );
        this.verticalElevatorMotorFollower = new TalonFX(
                elevatorConstants.verticalFollowerMotorId(),
                elevatorConstants.verticalElevatorCANBus()
        );
        this.verticalElevatorEncoder = new CANcoder(
                elevatorConstants.verticalEncoderId(),
                elevatorConstants.verticalElevatorCANBus()
        );

        this.horizontalElevatorMotor = new TalonFX(
                elevatorConstants.horizontalMotorId(),
                elevatorConstants.horizontalElevatorCANBus()
        );

        this.verticalElevatorLimitSwitch = new DigitalInput(elevatorConstants.verticalLimitSwitchDIOChannel());
        this.horizontalElevatorRearLimitSwitch = new DigitalInput(elevatorConstants.horizontalLimitSwitchDIOChannel());

        this.verticalPositionVoltage = new PositionVoltage(0);
        this.verticalMotionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0, 0);
        this.verticalDutyCycleOut = new DutyCycleOut(0);

        this.horizontalDutyCycleOut = new DutyCycleOut(0);
        this.horizontalMotionMagicVoltage = new MotionMagicVoltage(0);

        this._verticalPosition = verticalElevatorMotor.getPosition();
        this._verticalVelocity = verticalElevatorMotor.getVelocity();
        this._verticalDutyCycle = verticalElevatorMotor.getDutyCycle();
        this._verticalMotorTorqueCurrent = verticalElevatorMotor.getTorqueCurrent();
        this._verticalMotorFollowerTorqueCurrent = verticalElevatorMotorFollower.getTorqueCurrent();
        this._verticalMotorDeviceTemp = verticalElevatorMotor.getDeviceTemp();
        this._verticalMotorFollowerDeviceTemp = verticalElevatorMotorFollower.getDeviceTemp();
        this._horizontalPosition = horizontalElevatorMotor.getPosition();
        this._horizontalVelocity = horizontalElevatorMotor.getVelocity();
    }

    private DynamicMotionMagicVoltage getVerticalMotionMagicControl(
            final DynamicMotionMagicVoltage motionMagicVoltage,
            final double positionRots
    ) {
        final SuperstructureStates.VerticalTransitionMode verticalTransitionMode =
                SuperstructureStates.getVerticalTransitionMode(
                        verticalElevatorMode,
                        lastDesiredState.getVEControlInput(),
                        VEControlInput
                );

        return switch (verticalTransitionMode) {
            case HOLDING_Z, EXTENDING_Z_PLUS -> motionMagicVoltage
                    .withPosition(positionRots)
                    .withVelocity(100)
                    .withAcceleration(160)
                    .withJerk(200);
            case RETRACTING_Z_MINUS -> motionMagicVoltage
                    .withPosition(positionRots)
                    .withVelocity(20)
                    .withAcceleration(40)
                    .withJerk(60);
        };
    }

    private boolean resetElevator() {
        if (!verticalElevatorReset && verticalElevatorLimitSwitch.get()) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = SuperstructureStates.VerticalElevatorMode.DUTY_CYCLE;
            VEControlInput = 0;
            verticalElevatorReset = true;
        }

        if (!horizontalElevatorReset && horizontalElevatorRearLimitSwitch.get()) {
//            horizontalElevatorEncoder.setPosition(0);
            horizontalElevatorMotor.setPosition(0);
            horizontalElevatorMode = SuperstructureStates.HorizontalElevatorMode.DUTY_CYCLE;
            HEControlInput = 0;
            horizontalElevatorReset = true;
        }

        return verticalElevatorReset && horizontalElevatorReset;
    }

    private double getVEPosition() {
        return Phoenix6Utils.latencyCompensateRefreshedSignalIfIsGood(_verticalPosition, _verticalVelocity);
    }

    private double getHEPosition() {
        return Phoenix6Utils.latencyCompensateRefreshedSignalIfIsGood(_horizontalPosition, _horizontalVelocity);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                _verticalPosition,
                _verticalVelocity,
                _verticalDutyCycle,
                _verticalMotorTorqueCurrent,
                _verticalMotorFollowerTorqueCurrent,
                _verticalMotorDeviceTemp,
                _verticalMotorFollowerDeviceTemp,
                _horizontalPosition,
                _horizontalVelocity
        );

        if (desiredState == SuperstructureStates.ElevatorState.ELEVATOR_RESET && !elevatorsHaveReset) {
            elevatorsHaveReset = resetElevator();
            if (elevatorsHaveReset) {
                setDesiredState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY);
            }
        }

        switch (verticalElevatorMode) {
            case POSITION -> verticalElevatorMotor.setControl(
                    verticalPositionVoltage.withPosition(VEControlInput)
            );
            case MOTION_MAGIC -> verticalElevatorMotor.setControl(
                    getVerticalMotionMagicControl(verticalMotionMagicVoltage, VEControlInput)
            );
            case DUTY_CYCLE -> verticalElevatorMotor.setControl(
                    verticalDutyCycleOut.withOutput(VEControlInput)
            );
        }

        switch (horizontalElevatorMode) {
            case POSITION -> horizontalElevatorMotor.setControl(
                    horizontalMotionMagicVoltage.withPosition(HEControlInput)
            );
            case DUTY_CYCLE -> horizontalElevatorMotor.setControl(
                    horizontalDutyCycleOut.withOutput(HEControlInput)
            );
        }
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        inputs.verticalEncoderPositionRots = getVEPosition();
        inputs.verticalEncoderVelocityRotsPerSec = _verticalVelocity.getValue();
        inputs.verticalMotorDutyCycle = _verticalDutyCycle.getValue();
        inputs.verticalMotorCurrentsAmps = new double[] {
                _verticalMotorTorqueCurrent.getValue(),
                _verticalMotorFollowerTorqueCurrent.getValue()
        };
        inputs.verticalMotorTempsCelsius = new double[] {
                _verticalMotorDeviceTemp.getValue(),
                _verticalMotorFollowerDeviceTemp.getValue()
        };

        inputs.horizontalEncoderPositionRots = getHEPosition();
        inputs.horizontalEncoderVelocityRotsPerSec = _horizontalVelocity.getValue();
        inputs.horizontalMotorCurrentAmps = horizontalElevatorMotor.getTorqueCurrent().getValue();
        inputs.horizontalMotorDutyCycle = horizontalElevatorMotor.getDutyCycle().getValue();
        inputs.horizontalMotorTempCelsius = horizontalElevatorMotor.getDeviceTemp().getValue();

        inputs.verticalLimitSwitch = verticalElevatorLimitSwitch.get();
        inputs.horizontalLimitSwitch = horizontalElevatorRearLimitSwitch.get();

        inputs.elevatorsHaveReset = elevatorsHaveReset;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        // Vertical elevator CANCoder
        final CANcoderConfiguration verticalElevatorEncoderConfig = new CANcoderConfiguration();
        verticalElevatorEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        verticalElevatorEncoder.getConfigurator().apply(verticalElevatorEncoderConfig);

        // Vertical elevator motor
        final TalonFXConfiguration verticalElevatorMotorConfig = new TalonFXConfiguration();
        verticalElevatorMotorConfig.Slot0 = new Slot0Configs()
                .withKP(22)
                .withKG(0.6)
                .withGravityType(GravityTypeValue.Elevator_Static);

        verticalElevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        verticalElevatorMotorConfig.Feedback.RotorToSensorRatio = SimConstants.Elevator.Vertical.GEARING;
        verticalElevatorMotorConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();
        verticalElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        verticalElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        verticalElevatorMotor.getConfigurator().apply(verticalElevatorMotorConfig);

        // Vertical elevator motor follower
        final TalonFXConfiguration verticalElevatorMotorFollowerConfig = new TalonFXConfiguration();
        verticalElevatorMotorFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        verticalElevatorMotorFollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        verticalElevatorMotorFollower.getConfigurator().apply(verticalElevatorMotorFollowerConfig);
        verticalElevatorMotorFollower.setControl(new Follower(
                verticalElevatorMotor.getDeviceID(),
                false
        ));

        // Horizontal elevator motor
        final TalonFXConfiguration horizontalElevatorMotorConfig = new TalonFXConfiguration();
        horizontalElevatorMotorConfig.Slot0 = new Slot0Configs()
                .withKP(18);
        horizontalElevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        horizontalElevatorMotorConfig.Feedback.SensorToMechanismRatio = SimConstants.Elevator.Horizontal.GEARING;
        // TODO: we use the internal RotorSensor for the horizontal elevator because our CAN chain sucks, maybe we can
        //  use the CANCoder again if it ever gets fixed?
//        horizontalElevatorMotorConfig.Feedback.FeedbackRemoteSensorID = horizontalElevatorEncoder.getDeviceID();
        horizontalElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        horizontalElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        horizontalElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        horizontalElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        // TODO: tune motion magic config
        horizontalElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        horizontalElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 110;
        horizontalElevatorMotorConfig.MotionMagic.MotionMagicJerk = 165;

        horizontalElevatorMotor.getConfigurator().apply(horizontalElevatorMotorConfig);
    }

    @Override
    public void setDesiredState(final SuperstructureStates.ElevatorState desiredState) {
        this.lastDesiredState = this.desiredState;
        this.desiredState = desiredState;

        this.verticalElevatorMode = desiredState.getVerticalElevatorMode();
        this.VEControlInput = desiredState.getVEControlInput();
        this.horizontalElevatorMode = desiredState.getHorizontalElevatorMode();
        this.HEControlInput = desiredState.getHEControlInput();
    }
}
