package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.LimitSwitchSim;
import frc.robot.utils.sim.SimUtils;

public class ElevatorIOSimFalcon implements ElevatorIO {
    private final ElevatorSimSolver elevatorSimSolver;
    private final DeltaTime deltaTime;

    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final TalonFX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final LimitSwitchSim verticalElevatorLimitSwitchSim, horizontalElevatorRearLimitSwitchSim;

    private SuperstructureStates.ElevatorState desiredState = SuperstructureStates.ElevatorState.ELEVATOR_RESET;

    private final PositionVoltage verticalPositionVoltage;
    private final MotionMagicVoltage verticalMotionMagicVoltage;
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

    private SuperstructureStates.HorizontalElevatorMode horizontalElevatorMode = desiredState.getHorizontalElevatorMode();
    /**
     * Horizontal Elevator Control Input
     * <p>Units can be PositionRots, DutyCycle</p>
     */
    private double HEControlInput = desiredState.getHEControlInput();
    private SuperstructureStates.VerticalElevatorMode verticalElevatorMode = desiredState.getVerticalElevatorMode();
    /**
     * Vertical Elevator Control Input
     * <p>Units can be PositionRots, DutyCycle</p>
     * @see SuperstructureStates.VerticalElevatorMode
     */
    private double VEControlInput = desiredState.getVEControlInput();

    private boolean verticalElevatorReset = false;
    private boolean horizontalElevatorReset = false;
    private boolean elevatorsHaveReset = false;

    public ElevatorIOSimFalcon(final HardwareConstants.ElevatorConstants elevatorConstants) {
        this.deltaTime = new DeltaTime();

        // Vertical Elevator
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

        this.verticalElevatorLimitSwitchSim = new LimitSwitchSim(
                new DigitalInput(elevatorConstants.verticalLimitSwitchDIOChannel())
        );
        this.verticalElevatorLimitSwitchSim.setInitialized(true);

        // Horizontal Elevator
        this.horizontalElevatorMotor = new TalonFX(
                elevatorConstants.horizontalMotorId(),
                elevatorConstants.horizontalElevatorCANBus()
        );
        this.horizontalElevatorEncoder = new CANcoder(
                elevatorConstants.horizontalEncoderId(),
                elevatorConstants.horizontalElevatorCANBus()
        );

        this.horizontalElevatorRearLimitSwitchSim = new LimitSwitchSim(
                new DigitalInput(elevatorConstants.horizontalLimitSwitchDIOChannel())
        );
        this.verticalElevatorLimitSwitchSim.setInitialized(true);

        // Sim Solver
        this.elevatorSimSolver = new ElevatorSimSolver(
                verticalElevatorMotor,
                verticalElevatorMotorFollower,
                verticalElevatorEncoder,
                horizontalElevatorMotor,
                horizontalElevatorEncoder
        );

        this.verticalPositionVoltage = new PositionVoltage(0);
        this.verticalMotionMagicVoltage = new MotionMagicVoltage(0);
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

    private boolean resetElevator() {
        if (!verticalElevatorReset && verticalElevatorLimitSwitchSim.getValue()) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = SuperstructureStates.VerticalElevatorMode.DUTY_CYCLE;
            VEControlInput = 0;
            verticalElevatorReset = true;
        }

        if (!horizontalElevatorReset && horizontalElevatorRearLimitSwitchSim.getValue()) {
            horizontalElevatorMotor.setPosition(0);
            horizontalElevatorMode = SuperstructureStates.HorizontalElevatorMode.DUTY_CYCLE;
            HEControlInput = 0;
            horizontalElevatorReset = true;
        }

        return verticalElevatorReset && horizontalElevatorReset;
    }

    private double getVEPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                _verticalPosition,
                _verticalVelocity
        );
    }

    private double getHEPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                _horizontalPosition,
                _horizontalVelocity
        );
    }

    private void updateSimulation() {
        elevatorSimSolver.update(deltaTime.get());
        // TODO: do these checks need to be <= 0 instead of == 0? that could cause us to miss a -0.25 initialize again
        verticalElevatorLimitSwitchSim.setValue(getVEPosition() == 0);
        horizontalElevatorRearLimitSwitchSim.setValue(getHEPosition() == 0);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic() {
        updateSimulation();

        if (desiredState == SuperstructureStates.ElevatorState.ELEVATOR_RESET && !elevatorsHaveReset) {
            elevatorsHaveReset = resetElevator();
            if (elevatorsHaveReset) {
                // TODO: this isn't reported up to the subsystem layer as it happens within the IO layer
                //  probably needs to be addressed
                //  - also true for the direct sets found in resetElevator
                setDesiredState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY);
            }
        }

        switch (verticalElevatorMode) {
            case POSITION -> verticalElevatorMotor.setControl(
                    verticalPositionVoltage.withPosition(VEControlInput)
            );
            case MOTION_MAGIC -> verticalElevatorMotor.setControl(
                    verticalMotionMagicVoltage.withPosition(VEControlInput)
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
        final Elevator.ElevatorPoseState simState = elevatorSimSolver.getElevatorPoseState();
        simState.log(Elevator.logKey + "SimState");

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
        inputs.horizontalMotorCurrentAmps = horizontalElevatorMotor.getTorqueCurrent().refresh().getValue();
        inputs.horizontalMotorDutyCycle = horizontalElevatorMotor.getDutyCycle().refresh().getValue();
        inputs.horizontalMotorTempCelsius = horizontalElevatorMotor.getDeviceTemp().refresh().getValue();

        inputs.verticalLimitSwitch = verticalElevatorLimitSwitchSim.getValue();
        inputs.horizontalLimitSwitch = horizontalElevatorRearLimitSwitchSim.getValue();

        inputs.elevatorsHaveReset = elevatorsHaveReset;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        // Vertical Elevator
        final SensorDirectionValue verticalElevatorEncoderSensorDirection =
                SensorDirectionValue.CounterClockwise_Positive;
        final CANcoderConfiguration verticalElevatorEncoderConfig = new CANcoderConfiguration();
        verticalElevatorEncoderConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderSensorDirection;
        verticalElevatorEncoder.getConfigurator().apply(verticalElevatorEncoderConfig);

        SimUtils.setCTRECANCoderSimStateSensorDirection(
                verticalElevatorEncoder, verticalElevatorEncoderSensorDirection
        );
        // TODO: this fix for CANCoder initialization in sim doesn't seem to work all the time...investigate!
//        Phoenix6Utils.assertIsOK(SimUtils.initializeCTRECANCoderSim(verticalElevatorEncoder));

        final InvertedValue verticalElevatorMotorInverted = InvertedValue.Clockwise_Positive;
        final TalonFXConfiguration verticalElevatorMotorConfig = new TalonFXConfiguration();
        verticalElevatorMotorConfig.Slot0 = new Slot0Configs()
                .withKP(22);

        verticalElevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        verticalElevatorMotorConfig.Feedback.RotorToSensorRatio = SimConstants.Elevator.Vertical.GEARING;
        verticalElevatorMotorConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();

        verticalElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        verticalElevatorMotorConfig.MotorOutput.Inverted = verticalElevatorMotorInverted;

        verticalElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
        verticalElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 40;
        verticalElevatorMotorConfig.MotionMagic.MotionMagicJerk = 100;

        verticalElevatorMotor.getConfigurator().apply(verticalElevatorMotorConfig);

        final InvertedValue verticalElevatorMotorFollowerInverted = InvertedValue.Clockwise_Positive;
        final TalonFXConfiguration verticalElevatorMotorFollowerConfig = new TalonFXConfiguration();
        verticalElevatorMotorFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        verticalElevatorMotorFollowerConfig.MotorOutput.Inverted = verticalElevatorMotorFollowerInverted;

        final Follower verticalElevatorFollower = new Follower(
                verticalElevatorMotor.getDeviceID(),
                false
        );

        verticalElevatorMotorFollower.getConfigurator().apply(verticalElevatorMotorFollowerConfig);
        verticalElevatorMotorFollower.setControl(verticalElevatorFollower);

        SimUtils.setCTRETalonFXSimStateMotorInverted(
                verticalElevatorMotor, verticalElevatorMotorInverted
        );

        SimUtils.setCTRETalonFXSimStateMotorInverted(
                verticalElevatorMotorFollower, verticalElevatorMotorFollowerInverted
        );

        // Horizontal Elevator
        final SensorDirectionValue horizontalElevatorEncoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        final CANcoderConfiguration horizontalElevatorEncoderConfig = new CANcoderConfiguration();
        horizontalElevatorEncoderConfig.MagnetSensor.SensorDirection = horizontalElevatorEncoderSensorDirection;
        horizontalElevatorEncoder.getConfigurator().apply(horizontalElevatorEncoderConfig);

        SimUtils.setCTRECANCoderSimStateSensorDirection(
                horizontalElevatorEncoder, horizontalElevatorEncoderSensorDirection
        );
        // TODO: this fix for CANCoder initialization in sim doesn't seem to work all the time...investigate!
//        Phoenix6Utils.assertIsOK(SimUtils.initializeCTRECANCoderSim(horizontalElevatorEncoder));

        // Horizontal elevator motor
        final TalonFXConfiguration horizontalElevatorMotorConfig = new TalonFXConfiguration();
        horizontalElevatorMotorConfig.Slot0 = new Slot0Configs()
                .withKP(18);
        horizontalElevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        horizontalElevatorMotorConfig.Feedback.SensorToMechanismRatio = SimConstants.Elevator.Horizontal.GEARING;
        horizontalElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        horizontalElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        horizontalElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        horizontalElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        horizontalElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        horizontalElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 110;
        horizontalElevatorMotorConfig.MotionMagic.MotionMagicJerk = 165;

        horizontalElevatorMotor.getConfigurator().apply(horizontalElevatorMotorConfig);    }

    @Override
    public void setDesiredState(final SuperstructureStates.ElevatorState state) {
        this.desiredState = state;
        this.verticalElevatorMode = state.getVerticalElevatorMode();
        this.VEControlInput = state.getVEControlInput();
        this.horizontalElevatorMode = state.getHorizontalElevatorMode();
        this.HEControlInput = state.getHEControlInput();
    }
}
