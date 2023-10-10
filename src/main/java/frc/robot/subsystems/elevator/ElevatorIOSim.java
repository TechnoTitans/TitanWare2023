package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
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

    private final MotionMagicConfigs verticalExtensionMotionMagicConfig;
    private final MotionMagicConfigs verticalRetractionMotionMagicConfig;

    private final PositionVoltage positionVoltage;
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;

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

        config();

        this.horizontalElevatorPID = new ProfiledPIDController(
                0.3, 0, 0,
                new TrapezoidProfile.Constraints(10, 20)
        );

        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                horizontalElevatorPID,
                horizontalElevatorEncoder.getPosition().waitForUpdate(0.25),
                horizontalElevatorEncoder.getVelocity().waitForUpdate(0.25)
        );

        this.verticalExtensionMotionMagicConfig = new MotionMagicConfigs();
        verticalExtensionMotionMagicConfig.MotionMagicCruiseVelocity = 12;
        verticalExtensionMotionMagicConfig.MotionMagicAcceleration = 55;
        verticalExtensionMotionMagicConfig.MotionMagicJerk = 100;

        this.verticalRetractionMotionMagicConfig = new MotionMagicConfigs();
        verticalRetractionMotionMagicConfig.MotionMagicCruiseVelocity = 8;
        verticalRetractionMotionMagicConfig.MotionMagicAcceleration = 30;
        verticalRetractionMotionMagicConfig.MotionMagicJerk = 80;

        this.positionVoltage = new PositionVoltage(0);
        this.dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(
                0,
                verticalExtensionMotionMagicConfig.MotionMagicCruiseVelocity,
                verticalExtensionMotionMagicConfig.MotionMagicAcceleration,
                verticalExtensionMotionMagicConfig.MotionMagicJerk
        );
        this.dutyCycleOut = new DutyCycleOut(0);
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
                    horizontalElevatorEncoder.getPosition().waitForUpdate(0.25),
                    horizontalElevatorEncoder.getVelocity().waitForUpdate(0.25)
            );
        }

        return verticalElevatorReset && horizontalElevatorReset;
    }

    private double getVEControl() {
        return switch (verticalElevatorMode) {
            case POSITION, MOTION_MAGIC -> getVEPosition();
            case DUTY_CYCLE -> verticalElevatorMotor.getDutyCycle().refresh().getValue();
        };
    }

    private double getVEPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                verticalElevatorEncoder.getPosition(),
                verticalElevatorEncoder.getVelocity()
        );
    }

    private double getHEPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                horizontalElevatorEncoder.getPosition(),
                horizontalElevatorEncoder.getVelocity()
        );
    }

    private void updateSimulation() {
        elevatorSimSolver.update(deltaTime.get());
        // TODO: do these checks need to be <= 0 instead of == 0? that could cause us to miss a -0.25 initialize again
        verticalElevatorLimitSwitchSim.setValue(getVEPosition() == 0);
        horizontalElevatorRearLimitSwitchSim.setValue(getHEPosition() == 0);
    }

    private MotionMagicConfigs getVerticalMotionMagicConfig() {
        final SuperstructureStates.VerticalTransitionMode verticalTransitionMode =
                SuperstructureStates.getVerticalTransitionMode(verticalElevatorMode, getVEControl(), VEControlInput);

        return switch (verticalTransitionMode) {
            case EXTENDING_Z_PLUS -> verticalExtensionMotionMagicConfig;
            case RETRACTING_Z_MINUS -> verticalRetractionMotionMagicConfig;
        };
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
            case MOTION_MAGIC -> {
                final MotionMagicConfigs motionMagicConfig = getVerticalMotionMagicConfig();
                verticalElevatorMotor.setControl(
                        dynamicMotionMagicVoltage
                                .withPosition(VEControlInput)
                                .withVelocity(motionMagicConfig.MotionMagicCruiseVelocity)
                                .withAcceleration(motionMagicConfig.MotionMagicAcceleration)
                                .withJerk(motionMagicConfig.MotionMagicJerk)
                );
            }
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
                                    horizontalElevatorEncoder.getPosition().refresh().getValue(),
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
        inputs.verticalEncoderVelocityRotsPerSec = verticalElevatorEncoder.getVelocity().refresh().getValue();
        inputs.verticalMotorDutyCycle = verticalElevatorMotor.getDutyCycle().refresh().getValue();
        inputs.verticalMotorCurrentsAmps = new double[] {
                verticalElevatorMotor.getTorqueCurrent().refresh().getValue(),
                verticalElevatorMotorFollower.getTorqueCurrent().refresh().getValue()
        };
        inputs.verticalMotorTempsCelsius = new double[] {
                verticalElevatorMotor.getDeviceTemp().refresh().getValue(),
                verticalElevatorMotorFollower.getDeviceTemp().refresh().getValue()
        };

        inputs.horizontalEncoderPositionRots = getHEPosition();
        inputs.horizontalEncoderVelocityRotsPerSec = horizontalElevatorEncoder.getVelocity().refresh().getValue();
        inputs.horizontalMotorDutyCycle = horizontalElevatorMotor.getAppliedOutput();
        inputs.horizontalMotorTempCelsius = horizontalElevatorMotor.getMotorTemperature();

        inputs.verticalLimitSwitch = verticalElevatorLimitSwitchSim.getValue();
        inputs.horizontalLimitSwitch = horizontalElevatorRearLimitSwitchSim.getValue();
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
