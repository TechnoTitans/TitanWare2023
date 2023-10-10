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
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerInverted;
    private final SensorDirectionValue verticalElevatorEncoderR;
    private final TitanSparkMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorRearLimitSwitch;

    private SuperstructureStates.ElevatorState desiredState = SuperstructureStates.ElevatorState.ELEVATOR_RESET;
    private final ProfiledPIDController horizontalElevatorPID;

    private final MotionMagicConfigs verticalExtensionMotionMagicConfig;
    private final MotionMagicConfigs verticalRetractionMotionMagicConfig;

    private final PositionVoltage positionVoltage;
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;

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

        config();

        this.horizontalElevatorPID = new ProfiledPIDController(
                0.5, 0, 0,
                new TrapezoidProfile.Constraints(10, 13)
        );

        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                horizontalElevatorPID, horizontalElevatorEncoder.getPosition(), horizontalElevatorEncoder.getVelocity()
        );

        // TODO: tune motion magic configs for extension and retraction
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
        final boolean horizontalDidReset;
        final boolean verticalDidReset;

        if (verticalElevatorLimitSwitch.get()) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = SuperstructureStates.VerticalElevatorMode.DUTY_CYCLE;
            VEControlInput = 0;
            verticalDidReset = true;
        } else {
            verticalDidReset = false;
        }

        if (horizontalElevatorRearLimitSwitch.get()) {
            horizontalElevatorEncoder.setPosition(0);
            HEControlInput = 0;
            horizontalElevatorMode = SuperstructureStates.HorizontalElevatorMode.DUTY_CYCLE;
            horizontalDidReset = true;

            PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                    horizontalElevatorPID,
                    horizontalElevatorEncoder.getPosition().waitForUpdate(0.25),
                    horizontalElevatorEncoder.getVelocity().waitForUpdate(0.25)
            );
        } else {
            horizontalDidReset = false;
        }

        return verticalDidReset && horizontalDidReset;
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
        if (desiredState == SuperstructureStates.ElevatorState.ELEVATOR_RESET && !elevatorsHaveReset) {
            elevatorsHaveReset = resetElevator();
            if (elevatorsHaveReset) {
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
                    horizontalElevatorPID.calculate(
                            horizontalElevatorEncoder.getPosition().refresh().getValue(),
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
        verticalElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
        verticalElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 40;
        verticalElevatorMotorConfig.MotionMagic.MotionMagicJerk = 100;

        verticalElevatorMotor.getConfigurator().apply(verticalElevatorMotorConfig);

        // Vertical elevator motor follower
        final TalonFXConfiguration verticalElevatorMotorFollowerConfig = new TalonFXConfiguration();
        verticalElevatorMotorFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        verticalElevatorMotorFollowerConfig.MotorOutput.Inverted = verticalElevatorMotorFollowerInverted;

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
