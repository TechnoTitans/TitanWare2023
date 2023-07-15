package frc.robot.subsystems.elevator;

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
import frc.robot.utils.Enums;
import frc.robot.wrappers.api.Slot0Configs;
import frc.robot.wrappers.motors.TitanMAX;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerR;
    private final SensorDirectionValue verticalElevatorEncoderR;
    private final TitanMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorRearLimitSwitch, horizontalElevatorFrontLimitSwitch;

    private Enums.ElevatorState desiredState = Enums.ElevatorState.ELEVATOR_RESET;
    private final ProfiledPIDController horizontalElevatorPID;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;
    private Enums.ElevatorMode verticalElevatorMode;

    private double
            HEPositionRotations = 0, //Horizontal Elevator Target Rotations
            VEPositionRotations = 0; //Vertical Elevator Target Rotations

    private boolean elevatorsHaveReset = false;
    private boolean horizontalPositionalControl = false;

    public ElevatorIOReal(
            final TalonFX verticalElevatorMotor,
            final InvertedValue verticalElevatorMotorR,
            final TalonFX verticalElevatorMotorFollower,
            final InvertedValue verticalElevatorMotorFollowerR,
            final CANcoder verticalElevatorEncoder,
            final CANcoder horizontalElevatorEncoder,
            final SensorDirectionValue verticalElevatorEncoderR,
            final TitanMAX horizontalElevatorMotor,
            final DigitalInput verticalElevatorLimitSwitch,
            final DigitalInput horizontalElevatorRearLimitSwitch,
            final DigitalInput horizontalElevatorFrontLimitSwitch
    ) {
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorR = verticalElevatorMotorR;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerR = verticalElevatorMotorFollowerR;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.verticalElevatorEncoderR = verticalElevatorEncoderR;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;
        this.horizontalElevatorRearLimitSwitch = horizontalElevatorRearLimitSwitch;
        this.horizontalElevatorFrontLimitSwitch = horizontalElevatorFrontLimitSwitch;

        config();

        this.horizontalElevatorPID = new ProfiledPIDController(0.3, 0, 0,
                new TrapezoidProfile.Constraints(30, 40));

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicVoltage = new MotionMagicVoltage(0);
        this.dutyCycleOut = new DutyCycleOut(0);
    }

    private boolean resetElevator() {
        final boolean horizontalDidReset;
        final boolean verticalDidReset;

        if (verticalElevatorLimitSwitch.get()) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = Enums.ElevatorMode.DUTY_CYCLE;
            VEPositionRotations = 0;
            verticalDidReset = true;
        } else {
            verticalDidReset = false;
        }

        if (horizontalElevatorRearLimitSwitch.get()) {
            horizontalElevatorEncoder.setPosition(0);
            horizontalPositionalControl = true;
            HEPositionRotations = 0.1;
            horizontalDidReset = true;
        } else {
            horizontalDidReset = false;
        }

        return verticalDidReset && horizontalDidReset;
    }

    private void periodic() {
        final boolean hasHorizontalElevatorRearLSPressed = horizontalElevatorRearLimitSwitch.get();
        final boolean hasHorizontalElevatorFrontLimitSwitch = horizontalElevatorFrontLimitSwitch.get();
        final boolean hasVerticalElevatorLSPressed = verticalElevatorLimitSwitch.get();

        final double horizontalElevatorEncoderPositionRots = horizontalElevatorEncoder.getPosition().refresh().getValue();

        if (desiredState == Enums.ElevatorState.ELEVATOR_RESET && !elevatorsHaveReset) {
            elevatorsHaveReset = resetElevator();
            if (elevatorsHaveReset) {
                setDesiredState(Enums.ElevatorState.ELEVATOR_STANDBY);
            }
        }

        if (desiredState == Enums.ElevatorState.ELEVATOR_STANDBY &&
                horizontalElevatorEncoderPositionRots < 0.5 &&
                hasHorizontalElevatorRearLSPressed
        ) {
            horizontalPositionalControl = true;
            HEPositionRotations = 0.1;
        }

        if (desiredState == Enums.ElevatorState.ELEVATOR_RESET && hasHorizontalElevatorRearLSPressed) {
            horizontalElevatorEncoder.setPosition(0);
            horizontalPositionalControl = true;
            HEPositionRotations = 0.1;
        }

        if (desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH &&
                horizontalElevatorEncoderPositionRots > 1.5 &&
                hasHorizontalElevatorFrontLimitSwitch
        ) {
            horizontalPositionalControl = true;
            HEPositionRotations = horizontalElevatorEncoderPositionRots;
        }

        if (desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM && hasHorizontalElevatorRearLSPressed) {
            horizontalPositionalControl = true;
            HEPositionRotations = horizontalElevatorEncoderPositionRots;
        }

        if (desiredState == Enums.ElevatorState.ELEVATOR_STANDBY && hasVerticalElevatorLSPressed) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = Enums.ElevatorMode.DUTY_CYCLE;
            VEPositionRotations = 0;
        }

        switch (verticalElevatorMode) {
            case POSITION -> verticalElevatorMotor.setControl(
                    positionVoltage.withPosition(VEPositionRotations)
            );
            case MOTION_MAGIC -> verticalElevatorMotor.setControl(
                    motionMagicVoltage.withPosition(VEPositionRotations)
            );
            case DUTY_CYCLE -> verticalElevatorMotor.setControl(
                    dutyCycleOut.withOutput(VEPositionRotations)
            );
        }

        if (horizontalPositionalControl) {
            horizontalElevatorMotor.set(
                    CANSparkMax.ControlType.kDutyCycle,
                    horizontalElevatorPID.calculate(
                            horizontalElevatorEncoder.getPosition().refresh().getValue(),
                            HEPositionRotations
                    )
            );
        } else {
            horizontalElevatorMotor.set(
                    CANSparkMax.ControlType.kDutyCycle,
                    HEPositionRotations
            );
        }
    }

    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        periodic();

        inputs.desiredState = desiredState.toString();

        inputs.verticalElevatorMode = verticalElevatorMode.toString();
        inputs.VEPositionRotations = VEPositionRotations;

        inputs.horizontalPositionalControl = horizontalPositionalControl;
        inputs.HEPositionRotations = HEPositionRotations;

        inputs.verticalElevatorEncoderPosition = verticalElevatorEncoder.getPosition().refresh().getValue();
        inputs.verticalElevatorEncoderVelocity = verticalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.horizontalElevatorEncoderPosition = horizontalElevatorEncoder.getPosition().refresh().getValue();
        inputs.horizontalElevatorEncoderVelocity = horizontalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch.get();
        inputs.horizontalElevatorLimitSwitch = horizontalElevatorRearLimitSwitch.get();
    }

    @Override
    public void config() {
        final CANcoderConfiguration CVEConfig = new CANcoderConfiguration();
        CVEConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderR;
        verticalElevatorEncoder.getConfigurator().apply(CVEConfig);

        final TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0 = new Slot0Configs(9, 0.15, 0.15925, 1.4126);
        VEConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        VEConfig.Feedback.RotorToSensorRatio = 1/0.0938;
        VEConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();
        VEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEConfig.MotorOutput.Inverted = verticalElevatorMotorR;
        VEConfig.MotionMagic.MotionMagicCruiseVelocity = 75;
        VEConfig.MotionMagic.MotionMagicAcceleration = 25;
        VEConfig.MotionMagic.MotionMagicJerk = 40;
        verticalElevatorMotor.getConfigurator().apply(VEConfig);

        final TalonFXConfiguration VEFConfig = new TalonFXConfiguration();
        VEFConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEFConfig.MotorOutput.Inverted = verticalElevatorMotorFollowerR;
        verticalElevatorMotorFollower.getConfigurator().apply(VEFConfig);

        final Follower verticalElevatorFollower = new Follower(
                verticalElevatorMotor.getDeviceID(),
                false
        );
        verticalElevatorMotorFollower.setControl(verticalElevatorFollower);

        horizontalElevatorMotor.brake();

        final CANcoderConfiguration horizontalElevatorEncoderConfig = new CANcoderConfiguration();
        horizontalElevatorEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        horizontalElevatorEncoder.getConfigurator().apply(horizontalElevatorEncoderConfig);
    }

    @Override
    public void setDesiredState(final Enums.ElevatorState state) {
        this.desiredState = state;
        this.verticalElevatorMode = state.getVerticalElevatorMode();
        this.VEPositionRotations = state.getVEPositionRotations();
        this.horizontalPositionalControl = state.isHorizontalPositionalControl();
        this.HEPositionRotations = state.getHEPositionRotation();
    }

    @Override
    public Enums.ElevatorState getDesiredState() {
        return desiredState;
    }
}
