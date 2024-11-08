package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix5Utils;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.wrappers.motors.TitanSparkMAX;

@SuppressWarnings("FieldCanBeLocal")
public class ClawIOStateSpaceReal implements ClawIO {
    private final DeltaTime deltaTime;

    private final TalonSRX clawMainWheelBag, clawFollowerWheelBag;
    private final InvertType clawMainWheelBagInverted;
    private final TalonSRX clawOpenCloseMotor;
    private final InvertType clawOpenCloseMotorInverted;
    private final CANCoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanSparkMAX clawTiltNeo;

    // Cached StatusSignals
    private final StatusSignal<Double> _tiltPosition;
    private final StatusSignal<Double> _tiltVelocity;

    private TrapezoidProfile.State lastProfiledState = new TrapezoidProfile.State();
    // TODO: tune these trapezoidal constraints
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Units.degreesToRadians(260),
            Units.degreesToRadians(360)
    );

    private final LinearSystem<N2, N1, N1> tiltPlant;
    private final KalmanFilter<N2, N1, N1> tiltObserver;
    private final LinearQuadraticRegulator<N2, N1, N1> tiltController;
    private final LinearSystemLoop<N2, N1, N1> tiltSystemLoop;

    private SuperstructureStates.ClawOpenCloseControlMode openCloseControlMode;
    private SuperstructureStates.ClawTiltControlMode clawTiltControlMode;

    //Claw Intake Wheel Percent Output
    private double desiredIntakeWheelsPercentOutput;
    //Claw Tilt Control Input
    private double desiredTiltControlInput;
    //Claw Open Close Control Input
    private double desiredOpenCloseControlInput;

    public ClawIOStateSpaceReal(
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

        this._tiltPosition = clawTiltEncoder.getAbsolutePosition();
        this._tiltVelocity = clawTiltEncoder.getVelocity();

        this.deltaTime = new DeltaTime();

        this.tiltPlant = LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1),
                SimConstants.Claw.TILT_MOI,
                SimConstants.Claw.TILT_GEARING
        );

        this.tiltObserver = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                tiltPlant,
                // How accurate we think our model is, in radians and radians/sec
                VecBuilder.fill(0.015, 0.1),
                // How accurate we think our encoder position data is.
                VecBuilder.fill(0.005),
                Constants.LOOP_PERIOD_SECONDS
        );

        this.tiltController = new LinearQuadraticRegulator<>(
                tiltPlant,
                // qelms.
                // Position and velocity error tolerances, in radians and radians per second.
                // Decrease this to more heavily penalize state excursion, or make the controller behave more
                // aggressively. In this example we weight position much more highly than velocity, but
                // this can be tuned to balance the two.
                VecBuilder.fill(Units.degreesToRadians(0.01), Units.degreesToRadians(0.01)),
                // relms.
                // Control effort (voltage) tolerance.
                // Decrease this to more heavily penalize control effort, or make the controller less aggressive.
                // 12 is a good starting point because that is the (approximate) maximum voltage of a battery.
                VecBuilder.fill(12.0),
                Constants.LOOP_PERIOD_SECONDS
        );

        this.tiltSystemLoop = new LinearSystemLoop<>(
                tiltPlant,
                tiltController,
                tiltObserver,
                Constants.PDH.BATTERY_NOMINAL_VOLTAGE,
                Constants.LOOP_PERIOD_SECONDS
        );
    }

    public double getTiltAbsolutePositionRots() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_tiltPosition, _tiltVelocity);
    }

    @Override
    public void initialize() {
        final double tiltEncoderAbsolutePositionRads = Units.rotationsToRadians(getTiltAbsolutePositionRots());
        final double tiltEncoderVelocityRadsPerSec = Units.rotationsToRadians(
                _tiltVelocity.refresh().getValue()
        );

        tiltSystemLoop.reset(VecBuilder.fill(
                tiltEncoderAbsolutePositionRads,
                tiltEncoderVelocityRadsPerSec
        ));

        lastProfiledState = new TrapezoidProfile.State(
                tiltEncoderAbsolutePositionRads,
                tiltEncoderVelocityRadsPerSec
        );
    }

    @Override
    public void periodic() {
        final double dtSeconds = deltaTime.get();

        clawMainWheelBag.set(ControlMode.PercentOutput, desiredIntakeWheelsPercentOutput);
        clawOpenCloseMotor.set(
                openCloseControlMode.getControlMode(),
                Phoenix5Utils.getPhoenix6To5ControlInput(
                        openCloseControlMode.getControlMode(), desiredOpenCloseControlInput
                )
        );

        switch (clawTiltControlMode) {
            case POSITION -> {
                final TrapezoidProfile.State goal = new TrapezoidProfile.State(
                        Units.rotationsToRadians(desiredTiltControlInput),
                        0
                );
                final TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, lastProfiledState);

                lastProfiledState = profile.calculate(dtSeconds);

                tiltSystemLoop.setNextR(lastProfiledState.position, lastProfiledState.velocity);
                tiltSystemLoop.correct(
                        VecBuilder.fill(Units.rotationsToRadians(getTiltAbsolutePositionRots()))
                );
                tiltSystemLoop.predict(dtSeconds);

                final double nextInputVoltage = tiltSystemLoop.getU(0);
                clawTiltNeo.getPIDController().setReference(nextInputVoltage, CANSparkMax.ControlType.kVoltage);
            }
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

        inputs.openCloseEncoderPositionRots = clawOpenCloseEncoder.getAbsolutePosition();
        inputs.openCloseEncoderVelocityRotsPerSec = clawOpenCloseEncoder.getVelocity();
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

    @Override
    public void setDesiredState(final SuperstructureStates.ClawState desiredState) {
        this.desiredIntakeWheelsPercentOutput = desiredState.getIntakeWheelsPercentOutput();
        this.clawTiltControlMode = desiredState.getClawTiltControlMode();
        this.desiredTiltControlInput = desiredState.getTiltControlInput();
        this.openCloseControlMode = desiredState.getClawOpenCloseControlMode();
        this.desiredOpenCloseControlInput = desiredState.getOpenCloseControlInput();
    }
}
