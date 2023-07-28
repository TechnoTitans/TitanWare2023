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
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSimSolver;
import frc.robot.utils.Enums;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix5Utils;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.wrappers.motors.TitanSparkMAX;

import java.util.function.Supplier;

public class ClawIOSim implements ClawIO {
    private final ClawSimSolver clawSimSolver;

    private final TalonSRX clawMainWheelBag, clawFollowerWheelBag;
    private final InvertType clawMainWheelBagInverted;
    private final TalonSRX clawOpenCloseMotor;
    private final InvertType clawOpenCloseMotorInverted;
    private final CANCoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanSparkMAX clawTiltNeo;

    private final Supplier<ElevatorSimSolver.ElevatorSimState> elevatorSimStateSupplier;

    private final ProfiledPIDController tiltPID;

    private Enums.ClawOpenCloseControlMode openCloseControlMode;
    private Enums.ClawTiltControlMode clawTiltControlMode;

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
            final CANCoder clawOpenCloseEncoder,
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

        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;
        this.clawMainWheelBagInverted = clawMainWheelBagInverted;

        this.clawTiltNeo = clawTiltNeo;
        this.clawTiltEncoder = clawTiltEncoder;

        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        this.clawOpenCloseMotorInverted = clawOpenCloseMotorInverted;

        this.elevatorSimStateSupplier = elevatorSimStateSupplier;

        config();

        //TODO: tune pid
        this.tiltPID = new ProfiledPIDController(
                3, 0, 0,
                new TrapezoidProfile.Constraints(3, 5)
        );

        //TODO: something wrong with reset here I think
        // solved/mitigated for now (but untested) using waitForUpdate, check that this actually fixes the issue
        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                tiltPID,
                clawTiltEncoder.getAbsolutePosition().waitForUpdate(0.25),
                clawTiltEncoder.getVelocity().waitForUpdate(0.25)
        );
    }

    @Override
    public void periodic() {
        clawSimSolver.update(Constants.LOOP_PERIOD_SECONDS, elevatorSimStateSupplier.get());

        clawMainWheelBag.set(ControlMode.PercentOutput, desiredIntakeWheelsPercentOutput);

        final double controlInput = Phoenix5Utils.getPhoenix6To5ControlInput(
                openCloseControlMode.getControlMode(), desiredOpenCloseControlInput
        );

        clawOpenCloseMotor.set(
                openCloseControlMode.getControlMode(),
                controlInput
        );

        switch (clawTiltControlMode) {
            case POSITION -> clawTiltNeo.set(
                    CANSparkMax.ControlType.kVoltage,
                    RevUtils.convertControlTypeOutput(
                            clawTiltNeo,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            tiltPID.calculate(
                                    clawTiltEncoder.getAbsolutePosition().refresh().getValue(), desiredTiltControlInput
                            )
                    )
            );
            case DUTY_CYCLE -> clawTiltNeo.set(
                    CANSparkMax.ControlType.kVoltage,
                    RevUtils.convertControlTypeOutput(
                            clawTiltNeo,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            desiredTiltControlInput
                    )
            );
        }
    }

    @Override
    public void updateInputs(final ClawIO.ClawIOInputs inputs) {
        final ClawSimSolver.ClawSimState clawSimState = clawSimSolver.getClawSimState();
        clawSimState.log(Claw.logKey + "SimState");

        inputs.currentTiltPercentOutput = clawTiltNeo.getAppliedOutput();
        inputs.currentTiltEncoderPositionRots = clawTiltEncoder.getAbsolutePosition().refresh().getValue();
        inputs.currentTiltEncoderVelocityRotsPerSec = clawTiltEncoder.getVelocity().refresh().getValue();
        inputs.desiredTiltControlInput = desiredTiltControlInput;
        inputs.tiltCurrentAmps = clawSimSolver.getClawTiltSim().getCurrentDrawAmps();

        inputs.currentOpenClosePercentOutput = clawOpenCloseMotor.getMotorOutputPercent();
        inputs.currentOpenCloseEncoderPositionRots = clawOpenCloseEncoder.getAbsolutePosition();
        inputs.currentOpenCloseEncoderVelocityRotsPerSec = clawOpenCloseEncoder.getVelocity();
        inputs.desiredOpenCloseControlInput = desiredOpenCloseControlInput;
        inputs.openCloseCurrentAmps = clawOpenCloseMotor.getStatorCurrent();

        inputs.desiredIntakeWheelsPercentOutput = desiredIntakeWheelsPercentOutput;
        inputs.currentIntakeWheelsPercentOutput = clawMainWheelBag.getMotorOutputPercent();
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
        clawOpenCloseEncoderConfig.magnetOffsetDegrees = 0;

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

        // Claw Tilt Encoder
        final SensorDirectionValue clawTiltEncoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        final CANcoderConfiguration clawTiltEncoderConfig = new CANcoderConfiguration();
        clawTiltEncoderConfig.MagnetSensor.SensorDirection = clawTiltEncoderSensorDirection;
        clawTiltEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        clawTiltEncoderConfig.MagnetSensor.MagnetOffset = -0.25;

        SimUtils.setCTRECANCoderSimStateSensorDirection(clawTiltEncoder, clawTiltEncoderSensorDirection);

        clawTiltEncoder.getConfigurator().apply(clawTiltEncoderConfig);
    }

    public void setDesiredState(final Enums.ClawState state) {
        desiredIntakeWheelsPercentOutput = state.getIntakeWheelsPercentOutput();
        clawTiltControlMode = state.getClawTiltControlMode();
        desiredTiltControlInput = state.getTiltControlInput();
        openCloseControlMode = state.getClawOpenCloseControlMode();
        desiredOpenCloseControlInput = state.getOpenCloseControlInput();
    }
}
