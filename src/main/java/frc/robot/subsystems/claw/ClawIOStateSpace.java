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
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix5Utils;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class ClawIOStateSpace implements ClawIO {
    private final TalonSRX clawMainWheelBag, clawFollowerWheelBag;
    private final InvertType clawMainWheelBagInverted;
    private final TalonSRX clawOpenCloseMotor;
    private final InvertType clawOpenCloseMotorInverted;
    private final CANCoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanSparkMAX clawTiltNeo;

//    private final ProfiledPIDController tiltPID;
    private TrapezoidProfile.State lastProfiledState = new TrapezoidProfile.State();
    private final DeltaTime deltaTime;

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

    public ClawIOStateSpace(
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

        this.deltaTime = new DeltaTime();

        this.tiltPlant = LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1),
                Constants.Sim.Claw.TILT_MOI,
                Constants.Sim.Claw.TILT_GEARING
        );
        this.tiltObserver = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                tiltPlant,
                VecBuilder.fill(0.015, 0.17),
                VecBuilder.fill(0.01),
                Constants.LOOP_PERIOD_SECONDS
        );
        this.tiltController = new LinearQuadraticRegulator<>(
                tiltPlant,
                VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)),
                VecBuilder.fill(12.0),
                Constants.LOOP_PERIOD_SECONDS
        );
        this.tiltSystemLoop = new LinearSystemLoop<>(
                tiltPlant,
                tiltController,
                tiltObserver,
                12.0,
                Constants.LOOP_PERIOD_SECONDS
        );

        //TODO: tune pid in real
//        this.tiltPID = new ProfiledPIDController(
//                3, 0, 0,
//                new TrapezoidProfile.Constraints(3, 3)
//        );

//        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
//                tiltPID,
//                clawTiltEncoder.getAbsolutePosition().waitForUpdate(0.25),
//                clawTiltEncoder.getVelocity().waitForUpdate(0.25)
//        );

        final double tiltEncoderAbsolutePosition = clawTiltEncoder.getAbsolutePosition().refresh().getValue();
        final double tiltEncoderVelocity = clawTiltEncoder.getVelocity().refresh().getValue();

        tiltSystemLoop.reset(VecBuilder.fill(
                tiltEncoderAbsolutePosition,
                tiltEncoderVelocity
        ));

        lastProfiledState = new TrapezoidProfile.State(
                tiltEncoderAbsolutePosition,
                tiltEncoderVelocity
        );
    }

    @Override
    public void periodic() {
        clawMainWheelBag.set(ControlMode.PercentOutput, desiredIntakeWheelsPercentOutput);
        clawOpenCloseMotor.set(
                openCloseControlMode.getControlMode(),
                Phoenix5Utils.getPhoenix6To5ControlInput(
                        openCloseControlMode.getControlMode(), desiredOpenCloseControlInput
                )
        );

        final TrapezoidProfile.State goal = new TrapezoidProfile.State(desiredTiltControlInput, 0);
        final TrapezoidProfile profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        Units.degreesToRadians(5),
                        Units.degreesToRadians(5)
                ),
                goal,
                lastProfiledState
        );

        switch (clawTiltControlMode) {
            case POSITION -> {
                final double dtSeconds = deltaTime.get();
                lastProfiledState = profile.calculate(dtSeconds);

                tiltSystemLoop.setNextR(lastProfiledState.position, lastProfiledState.velocity);
                tiltSystemLoop.correct(VecBuilder.fill(clawTiltEncoder.getAbsolutePosition().refresh().getValue()));
                tiltSystemLoop.predict(dtSeconds);

                final double nextInputVoltage = tiltSystemLoop.getU(0);
                clawTiltNeo.getPIDController().setReference(nextInputVoltage, CANSparkMax.ControlType.kVoltage);
            }
            case DUTY_CYCLE -> throw new RuntimeException("UhOh!");
        }
    }

    @Override
    public void updateInputs(final ClawIOInputs inputs) {
        inputs.tiltEncoderPositionRots = clawTiltEncoder.getAbsolutePosition().refresh().getValue();
        inputs.tiltEncoderVelocityRotsPerSec = clawTiltEncoder.getVelocity().refresh().getValue();
        inputs.tiltCurrentAmps = clawTiltNeo.getOutputCurrent();

        inputs.openCloseEncoderPositionRots = clawOpenCloseEncoder.getAbsolutePosition();
        inputs.openCloseEncoderVelocityRotsPerSec = clawOpenCloseEncoder.getVelocity();
        inputs.openCloseCurrentAmps = clawOpenCloseMotor.getStatorCurrent();

        inputs.intakeWheelsPercentOutput = clawMainWheelBag.getMotorOutputPercent();
    }

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
        desiredIntakeWheelsPercentOutput = desiredState.getIntakeWheelsPercentOutput();
        clawTiltControlMode = desiredState.getClawTiltControlMode();
        desiredTiltControlInput = desiredState.getTiltControlInput();
        openCloseControlMode = desiredState.getClawOpenCloseControlMode();
        desiredOpenCloseControlInput = desiredState.getOpenCloseControlInput();
    }
}
