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
import frc.robot.utils.Enums;
import frc.robot.utils.control.PIDUtils;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.CTREPhoenix6TalonFXSim;
import frc.robot.utils.sim.SimUtils;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSimSolver elevatorSimSolver;

    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final CTREPhoenix6TalonFXSim verticalElevatorSimMotors;
    private final InvertedValue verticalElevatorMotorInverted, verticalElevatorMotorFollowerInverted;
    private final TitanSparkMAX horizontalElevatorMotor;
    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final SensorDirectionValue verticalElevatorEncoderSensorDirection, horizontalElevatorEncoderSensorDirection;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorRearLimitSwitch;

    private Enums.ElevatorState desiredState = Enums.ElevatorState.ELEVATOR_RESET;
    private final ProfiledPIDController horizontalElevatorPID;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;

    private final boolean isReal;

    private Enums.HorizontalElevatorMode horizontalElevatorMode = desiredState.getHorizontalElevatorMode();
    /**
     * Horizontal Elevator Control Input
     * <p>Units can be PositionRots, DutyCycle</p>
     */
    private double HEControlInput = desiredState.getHEControlInput();
    private Enums.VerticalElevatorMode verticalElevatorMode = desiredState.getVerticalElevatorMode();
    /**
     * Vertical Elevator Control Input
     * <p>Units can be PositionRots, DutyCycle</p>
     * @see Enums.VerticalElevatorMode
     */
    private double VEControlInput = desiredState.getVEControlInput(); //Vertical Elevator Target Rotations
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
            final DigitalInput horizontalElevatorRearLimitSwitch
    ) {
        this.elevatorSimSolver = new ElevatorSimSolver(
                verticalElevatorMotor,
                verticalElevatorMotorFollower,
                verticalElevatorEncoder,
                horizontalElevatorEncoder,
                horizontalElevatorMotor
        );

        // Vertical Elevator
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorInverted = verticalElevatorMotorInverted;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerInverted = verticalElevatorMotorFollowerInverted;

        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorEncoderSensorDirection = verticalElevatorEncoderSensorDirection;

        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;

        this.verticalElevatorSimMotors = elevatorSimSolver.getVerticalElevatorSimMotors();

        // Horizontal Elevator
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.horizontalElevatorEncoderSensorDirection = horizontalElevatorEncoderSensorDirection;

        this.horizontalElevatorRearLimitSwitch = horizontalElevatorRearLimitSwitch;

        config();

        this.horizontalElevatorPID = new ProfiledPIDController(0.3, 0, 0,
                new TrapezoidProfile.Constraints(10, 20)
        );

        PIDUtils.resetProfiledPIDControllerWithStatusSignal(
                horizontalElevatorPID,
                horizontalElevatorEncoder.getPosition().waitForUpdate(0.25),
                horizontalElevatorEncoder.getVelocity().waitForUpdate(0.25)
        );

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicVoltage = new MotionMagicVoltage(0);
        this.dutyCycleOut = new DutyCycleOut(0);

        this.isReal = Constants.CURRENT_MODE == Constants.RobotMode.REAL;
    }

    @Override
    public boolean isReal() {
        return isReal;
    }

    private boolean resetElevator() {
        final boolean horizontalDidReset;
        final boolean verticalDidReset;

        if (verticalElevatorLimitSwitch.get()) {
            verticalElevatorEncoder.setPosition(0);
            verticalElevatorMode = Enums.VerticalElevatorMode.DUTY_CYCLE;
            VEControlInput = 0;
            verticalDidReset = true;
        } else {
            verticalDidReset = false;
        }

        if (horizontalElevatorRearLimitSwitch.get()) {
            horizontalElevatorEncoder.setPosition(0);
            HEControlInput = 0;
            horizontalElevatorMode = Enums.HorizontalElevatorMode.DUTY_CYCLE;
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

    private void simulationPeriodic() {
        if (!isReal) {
            elevatorSimSolver.update(Constants.LOOP_PERIOD_SECONDS);
        }
    }

    @Override
    public void periodic() {
        simulationPeriodic();

        if (desiredState == Enums.ElevatorState.ELEVATOR_RESET && !elevatorsHaveReset) {
            elevatorsHaveReset = resetElevator();
            if (elevatorsHaveReset) {
                setDesiredState(Enums.ElevatorState.ELEVATOR_STANDBY);
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
                    CANSparkMax.ControlType.kVoltage,
                    RevUtils.convertControlTypeOutput(
                            horizontalElevatorMotor,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            horizontalElevatorPID.calculate(
                                    horizontalElevatorEncoder.getPosition().refresh().getValue(),
                                    HEControlInput
                            )
                    )
            );
            case DUTY_CYCLE -> horizontalElevatorMotor.set(
                    CANSparkMax.ControlType.kVoltage,
                    RevUtils.convertControlTypeOutput(
                            horizontalElevatorMotor,
                            CANSparkMax.ControlType.kDutyCycle,
                            CANSparkMax.ControlType.kVoltage,
                            HEControlInput
                    )
            );
        }
    }

    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        final ElevatorSimSolver.ElevatorSimState simState = elevatorSimSolver.getElevatorSimState();
        simState.log("ElevatorSimState");

        inputs.desiredState = desiredState.toString();

        inputs.verticalElevatorMode = verticalElevatorMode.toString();
        inputs.VEControlInput = VEControlInput;

        inputs.horizontalElevatorMode = horizontalElevatorMode.toString();
        inputs.HEControlInput = HEControlInput;

        inputs.verticalElevatorEncoderPosition = verticalElevatorEncoder.getPosition().refresh().getValue();
        inputs.verticalElevatorEncoderVelocity = verticalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.horizontalElevatorEncoderPosition = horizontalElevatorEncoder.getPosition().refresh().getValue();
        inputs.horizontalElevatorEncoderVelocity = horizontalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch.get();
        inputs.horizontalElevatorLimitSwitch = horizontalElevatorRearLimitSwitch.get();
    }

    @Override
    public void config() {
        // Vertical Elevator
        final CANcoderConfiguration verticalElevatorEncoderConfig = new CANcoderConfiguration();
        verticalElevatorEncoderConfig.MagnetSensor.SensorDirection = verticalElevatorEncoderSensorDirection;
        verticalElevatorEncoder.getConfigurator().apply(verticalElevatorEncoderConfig);

        SimUtils.setCTRECANCoderSimStateSensorDirection(
                verticalElevatorEncoder, verticalElevatorEncoderSensorDirection
        );

        final TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0 = new Slot0Configs(22, 0, 0, 0);

        VEConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        VEConfig.Feedback.RotorToSensorRatio = Constants.Sim.Elevator.Vertical.GEARING;
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

        horizontalElevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void setDesiredState(final Enums.ElevatorState state) {
        this.desiredState = state;
        this.verticalElevatorMode = state.getVerticalElevatorMode();
        this.VEControlInput = state.getVEControlInput();
        this.horizontalElevatorMode = state.getHorizontalElevatorMode();
        this.HEControlInput = state.getHEControlInput();
    }

    @Override
    public Enums.ElevatorState getDesiredState() {
        return desiredState;
    }

    @Override
    public ElevatorSimSolver.ElevatorSimState getElevatorSimState() {
        return elevatorSimSolver.getElevatorSimState();
    }
}
