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
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.LimitSwitchSim;
import frc.robot.utils.sim.SimUtils;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSimSolver elevatorSimSolver;
    private final DeltaTime deltaTime;

    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorInverted, verticalElevatorMotorFollowerInverted;
    private final TitanSparkMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final SensorDirectionValue verticalElevatorEncoderSensorDirection, horizontalElevatorEncoderSensorDirection;
    private final LimitSwitchSim verticalElevatorLimitSwitchSim, horizontalElevatorRearLimitSwitchSim;

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

    public ElevatorIOSim(
            final TalonFX verticalElevatorMotor,
            final InvertedValue verticalElevatorMotorInverted,
            final TalonFX verticalElevatorMotorFollower,
            final InvertedValue verticalElevatorMotorFollowerInverted,
            final CANcoder verticalElevatorEncoder,
            final SensorDirectionValue verticalElevatorEncoderSensorDirection,
            final CANcoder horizontalElevatorEncoder,
            final SensorDirectionValue horizontalElevatorEncoderSensorDirection,
            final TitanSparkMAX horizontalElevatorMotor,
            final DigitalInput verticalElevatorLimitSwitch,
            final DigitalInput horizontalElevatorRearLimitSwitch,
            final ElevatorSimSolver elevatorSimSolver
    ) {
        this.elevatorSimSolver = elevatorSimSolver;
        this.deltaTime = new DeltaTime();

        // Vertical Elevator
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorInverted = verticalElevatorMotorInverted;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerInverted = verticalElevatorMotorFollowerInverted;

        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorEncoderSensorDirection = verticalElevatorEncoderSensorDirection;

        this.verticalElevatorLimitSwitchSim = new LimitSwitchSim(verticalElevatorLimitSwitch);
        this.verticalElevatorLimitSwitchSim.setInitialized(true);

        // Horizontal Elevator
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.horizontalElevatorEncoderSensorDirection = horizontalElevatorEncoderSensorDirection;

        this.horizontalElevatorRearLimitSwitchSim = new LimitSwitchSim(horizontalElevatorRearLimitSwitch);
        this.verticalElevatorLimitSwitchSim.setInitialized(true);

        this.horizontalElevatorPID = new ProfiledPIDController(
                0.3, 0, 0,
                new TrapezoidProfile.Constraints(10, 20)
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
        if (!verticalElevatorReset && verticalElevatorLimitSwitchSim.getValue()) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = SuperstructureStates.VerticalElevatorMode.DUTY_CYCLE;
            VEControlInput = 0;
            verticalElevatorReset = true;
        }

        if (!horizontalElevatorReset && horizontalElevatorRearLimitSwitchSim.getValue()) {
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
                    RevUtils.convertControlTypeOutput(
                            horizontalElevatorMotor,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            horizontalElevatorPID.calculate(
                                    _horizontalPosition.refresh().getValue(),
                                    HEControlInput
                            )
                    ),
                    CANSparkMax.ControlType.kVoltage
            );
            // TODO: we can likely just use kDutyCycle control here, im fairly sure it works in sim
            case DUTY_CYCLE -> horizontalElevatorMotor.getPIDController().setReference(
                    RevUtils.convertControlTypeOutput(
                            horizontalElevatorMotor,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            HEControlInput
                    ),
                    CANSparkMax.ControlType.kVoltage
            );
        }
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        final ElevatorSimSolver.ElevatorSimState simState = elevatorSimSolver.getElevatorSimState();
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
        inputs.horizontalMotorDutyCycle = horizontalElevatorMotor.getAppliedOutput();
        inputs.horizontalMotorTempCelsius = horizontalElevatorMotor.getMotorTemperature();

        inputs.verticalLimitSwitch = verticalElevatorLimitSwitchSim.getValue();
        inputs.horizontalLimitSwitch = horizontalElevatorRearLimitSwitchSim.getValue();

        inputs.elevatorsHaveReset = elevatorsHaveReset;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        // Vertical Elevator
        final CANcoderConfiguration verticalElevatorEncoderConfig = new CANcoderConfiguration();
        verticalElevatorEncoderConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderSensorDirection;
        verticalElevatorEncoder.getConfigurator().apply(verticalElevatorEncoderConfig);

        SimUtils.setCTRECANCoderSimStateSensorDirection(
                verticalElevatorEncoder, verticalElevatorEncoderSensorDirection
        );
        // TODO: this fix for CANCoder initialization in sim doesn't seem to work all the time...investigate!
//        Phoenix6Utils.assertIsOK(SimUtils.initializeCTRECANCoderSim(verticalElevatorEncoder));

        final TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0 = new Slot0Configs(22, 0, 0, 0);

        VEConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        VEConfig.Feedback.RotorToSensorRatio = SimConstants.Elevator.Vertical.GEARING;
        VEConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();

        VEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEConfig.MotorOutput.Inverted = verticalElevatorMotorInverted;

        VEConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
        VEConfig.MotionMagic.MotionMagicAcceleration = 40;
        VEConfig.MotionMagic.MotionMagicJerk = 100;

        verticalElevatorMotor.getConfigurator().apply(VEConfig);

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
        final CANcoderConfiguration horizontalElevatorEncoderConfig = new CANcoderConfiguration();
        horizontalElevatorEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        horizontalElevatorEncoder.getConfigurator().apply(horizontalElevatorEncoderConfig);

        SimUtils.setCTRECANCoderSimStateSensorDirection(
                horizontalElevatorEncoder, horizontalElevatorEncoderSensorDirection
        );
        // TODO: this fix for CANCoder initialization in sim doesn't seem to work all the time...investigate!
//        Phoenix6Utils.assertIsOK(SimUtils.initializeCTRECANCoderSim(horizontalElevatorEncoder));

        horizontalElevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void setDesiredState(final SuperstructureStates.ElevatorState state) {
        this.desiredState = state;
        this.verticalElevatorMode = state.getVerticalElevatorMode();
        this.VEControlInput = state.getVEControlInput();
        this.horizontalElevatorMode = state.getHorizontalElevatorMode();
        this.HEControlInput = state.getHEControlInput();
    }
}
