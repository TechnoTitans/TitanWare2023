package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.elevator.ElevatorSimSolver;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix5Utils;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.wrappers.motors.TitanSparkMAX;

import java.util.function.Supplier;

public class ClawIOSim implements ClawIO {
    private final ClawSimSolver clawSimSolver;
    private final DeltaTime deltaTime;

    private final TalonSRX clawMainWheelBag, clawFollowerWheelBag;
    private final InvertType clawMainWheelBagInverted;
    private final TalonSRX clawOpenCloseMotor;
    private final InvertType clawOpenCloseMotorInverted;
    private final CANcoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanSparkMAX clawTiltNeo;

    private final Supplier<ElevatorSimSolver.ElevatorSimState> elevatorSimStateSupplier;

    private final ProfiledPIDController tiltPID;

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

    public ClawIOSim(
            final TalonSRX clawMainWheelBag,
            final TalonSRX clawFollowerWheelBag,
            final InvertType clawMainWheelBagInverted,
            final TalonSRX clawOpenCloseMotor,
            final InvertType clawOpenCloseMotorInverted,
            final CANcoder clawOpenCloseEncoder,
            final TitanSparkMAX clawTiltNeo,
            final CANcoder clawTiltEncoder,
            final Supplier<ElevatorSimSolver.ElevatorSimState> elevatorSimStateSupplier
    ) {
        this.clawSimSolver = new ClawSimSolver(
                clawMainWheelBag,
                clawFollowerWheelBag,
                clawOpenCloseMotor,
                clawOpenCloseEncoder,
                clawTiltNeo,
                clawTiltEncoder
        );
        this.deltaTime = new DeltaTime();

        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawMainWheelBagInverted = clawMainWheelBagInverted;

        this.clawTiltNeo = clawTiltNeo;
        this.clawTiltEncoder = clawTiltEncoder;

        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        this.clawOpenCloseMotorInverted = clawOpenCloseMotorInverted;

        this.elevatorSimStateSupplier = elevatorSimStateSupplier;

        //TODO: transfer tuned pid from real, then tune in sim
        this.tiltPID = new ProfiledPIDController(
                3, 0, 0,
                new TrapezoidProfile.Constraints(3, 5)
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
        final ElevatorSimSolver.ElevatorSimState elevatorSimState = elevatorSimStateSupplier.get();
        clawSimSolver.update(deltaTime.get(), elevatorSimState);

        clawMainWheelBag.set(ControlMode.PercentOutput, desiredIntakeWheelsPercentOutput);

        final double controlInput = Phoenix5Utils.getPhoenix6To5ControlInput(
                openCloseControlMode.getControlMode(), desiredOpenCloseControlInput
        );

        clawOpenCloseMotor.set(
                openCloseControlMode.getControlMode(),
                controlInput
        );

        switch (clawTiltControlMode) {
            case POSITION -> clawTiltNeo.getPIDController().setReference(
                    RevUtils.convertControlTypeOutput(
                            clawTiltNeo,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            tiltPID.calculate(
                                    _tiltPosition.refresh().getValue(),
                                    desiredTiltControlInput
                            )
                    ),
                    CANSparkMax.ControlType.kVoltage
            );
            case DUTY_CYCLE -> clawTiltNeo.getPIDController().setReference(
                    RevUtils.convertControlTypeOutput(
                            clawTiltNeo,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            desiredTiltControlInput
                    ),
                    CANSparkMax.ControlType.kVoltage
            );
        }
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final ClawIO.ClawIOInputs inputs) {
        final ClawSimSolver.ClawSimState clawSimState = clawSimSolver.getClawSimState();
        clawSimState.log(Claw.logKey + "SimState");

        inputs.tiltEncoderPositionRots = _tiltPosition.refresh().getValue();
        inputs.tiltEncoderVelocityRotsPerSec = _tiltVelocity.refresh().getValue();
        inputs.tiltPercentOutput = clawTiltNeo.getAppliedOutput();
        // TODO: this doesn't work... probably cause adding static friction makes a plant non-linear
        inputs.tiltCurrentAmps = clawSimSolver.getClawTiltSim().getCurrentDrawAmps();
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
        clawMainWheelBag.setInverted(clawMainWheelBagInverted);

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

        // Claw Tilt Encoder
        final SensorDirectionValue clawTiltEncoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        final CANcoderConfiguration clawTiltEncoderConfig = new CANcoderConfiguration();
        clawTiltEncoderConfig.MagnetSensor.SensorDirection = clawTiltEncoderSensorDirection;
        //todo this should prob be unsigned
        clawTiltEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // TODO: this -0.25 probably used to be a naive fix for the +-0.25 offset that CANCoders in sim initialize to,
        //  do we still need this here?
        clawTiltEncoderConfig.MagnetSensor.MagnetOffset = -0.25; //-0.25

        SimUtils.setCTRECANCoderSimStateSensorDirection(clawTiltEncoder, clawTiltEncoderSensorDirection);
        // TODO: this doesn't seem to work to reset the CANCoder after it starts offset by +-0.25, even though this
        //  solution works on other CANCoders? or does it?
//        SimUtils.initializeCTRECANCoderSim(clawTiltEncoder);

        clawTiltEncoder.getConfigurator().apply(clawTiltEncoderConfig);
    }

    @Override
    public void setDesiredState(final SuperstructureStates.ClawState state) {
        this.desiredIntakeWheelsPercentOutput = state.getIntakeWheelsPercentOutput();
        this.clawTiltControlMode = state.getClawTiltControlMode();
        this.desiredTiltControlInput = state.getTiltControlInput();
        this.openCloseControlMode = state.getClawOpenCloseControlMode();
        this.desiredOpenCloseControlInput = state.getOpenCloseControlInput();
    }
}
