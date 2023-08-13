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
import frc.robot.Constants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.control.PIDUtils;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerR;
    private final SensorDirectionValue verticalElevatorEncoderR;
    private final TitanSparkMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorRearLimitSwitch;

    private SuperstructureStates.ElevatorState desiredState = SuperstructureStates.ElevatorState.ELEVATOR_RESET;
    private final ProfiledPIDController horizontalElevatorPID;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;

    private SuperstructureStates.VerticalElevatorMode verticalElevatorMode = desiredState.getVerticalElevatorMode();
    private double VEControlInput = desiredState.getVEControlInput(); //Vertical Elevator Target Rotations
    private SuperstructureStates.HorizontalElevatorMode horizontalElevatorMode = desiredState.getHorizontalElevatorMode();
    private double HEControlInput = desiredState.getHEControlInput(); //Horizontal Elevator Target Rotations

    private boolean elevatorsHaveReset = false;

    public ElevatorIOReal(
            final TalonFX verticalElevatorMotor,
            final InvertedValue verticalElevatorMotorR,
            final TalonFX verticalElevatorMotorFollower,
            final InvertedValue verticalElevatorMotorFollowerR,
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
        this.verticalElevatorMotorFollowerR = verticalElevatorMotorFollowerR;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.verticalElevatorEncoderR = verticalElevatorEncoderR;
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;
        this.horizontalElevatorRearLimitSwitch = horizontalElevatorRearLimitSwitch;

        config();

        this.horizontalElevatorPID = new ProfiledPIDController(0.5, 0, 0,
                new TrapezoidProfile.Constraints(10, 13)
        );

        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                horizontalElevatorPID, horizontalElevatorEncoder.getPosition(), horizontalElevatorEncoder.getVelocity()
        );

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicVoltage = new MotionMagicVoltage(0);
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
            case MOTION_MAGIC -> verticalElevatorMotor.setControl(
                    motionMagicVoltage.withPosition(VEControlInput)
            );
            case DUTY_CYCLE -> verticalElevatorMotor.setControl(
                    dutyCycleOut.withOutput(VEControlInput)
            );
        }

        switch (horizontalElevatorMode) {
            case POSITION -> horizontalElevatorMotor.set(
                    CANSparkMax.ControlType.kDutyCycle,
                    horizontalElevatorPID.calculate(
                            horizontalElevatorEncoder.getPosition().refresh().getValue(),
                            HEControlInput
                    )
            );
            case DUTY_CYCLE -> horizontalElevatorMotor.set(
                    CANSparkMax.ControlType.kDutyCycle,
                    HEControlInput
            );
        }
    }

    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        inputs.verticalElevatorMotorDutyCycle = verticalElevatorMotor.getDutyCycle().refresh().getValue();
        inputs.verticalElevatorEncoderPosition = verticalElevatorEncoder.getPosition().refresh().getValue();
        inputs.verticalElevatorEncoderVelocity = verticalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.horizontalElevatorMotorDutyCycle = horizontalElevatorMotor.getAppliedOutput();
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
        VEConfig.Slot0 = new Slot0Configs(22, 0, 0, 0);
        VEConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        VEConfig.Feedback.RotorToSensorRatio = Constants.Sim.Elevator.Vertical.GEARING;
        VEConfig.Feedback.FeedbackRemoteSensorID = verticalElevatorEncoder.getDeviceID();
        VEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        VEConfig.MotorOutput.Inverted = verticalElevatorMotorR;
        VEConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
        VEConfig.MotionMagic.MotionMagicAcceleration = 40;
        VEConfig.MotionMagic.MotionMagicJerk = 100;
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

        horizontalElevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

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
