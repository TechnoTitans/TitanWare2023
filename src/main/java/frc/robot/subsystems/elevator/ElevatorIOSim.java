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
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.CTREPhoenix6TalonFXSim;
import frc.robot.utils.sim.LimitSwitchSim;
import frc.robot.utils.sim.SimUtils;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSimSolver elevatorSimSolver;

    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final CTREPhoenix6TalonFXSim verticalElevatorSimMotors;
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
    private double VEControlInput = desiredState.getVEControlInput(); //Vertical Elevator Target Rotations

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

        // Vertical Elevator
        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorInverted = verticalElevatorMotorInverted;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerInverted = verticalElevatorMotorFollowerInverted;

        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorEncoderSensorDirection = verticalElevatorEncoderSensorDirection;

        this.verticalElevatorLimitSwitchSim = new LimitSwitchSim(verticalElevatorLimitSwitch);
        this.verticalElevatorLimitSwitchSim.setInitialized(true);

        this.verticalElevatorSimMotors = elevatorSimSolver.getVerticalElevatorSimMotors();

        // Horizontal Elevator
        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.horizontalElevatorEncoderSensorDirection = horizontalElevatorEncoderSensorDirection;

        this.horizontalElevatorRearLimitSwitchSim = new LimitSwitchSim(horizontalElevatorRearLimitSwitch);
        this.verticalElevatorLimitSwitchSim.setInitialized(true);

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
        elevatorSimSolver.update(Constants.LOOP_PERIOD_SECONDS);
        // TODO: do these checks need to be <= 0 instead of == 0? that could cause us to miss a -0.25 initialize again
        verticalElevatorLimitSwitchSim.setValue(getVEPosition() == 0);
        horizontalElevatorRearLimitSwitchSim.setValue(getHEPosition() == 0);
    }

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
        simState.log(Elevator.logKey + "SimState");

        inputs.verticalElevatorMotorDutyCycle = verticalElevatorMotor.getDutyCycle().refresh().getValue();
        inputs.verticalElevatorEncoderPosition = getVEPosition();
        inputs.verticalElevatorEncoderVelocity = verticalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.horizontalElevatorMotorDutyCycle = horizontalElevatorMotor.getAppliedOutput();
        inputs.horizontalElevatorEncoderPosition = getHEPosition();
        inputs.horizontalElevatorEncoderVelocity = horizontalElevatorEncoder.getVelocity().refresh().getValue();

        inputs.verticalElevatorLimitSwitch = verticalElevatorLimitSwitchSim.getValue();
        inputs.horizontalElevatorLimitSwitch = horizontalElevatorRearLimitSwitchSim.getValue();
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
        Phoenix6Utils.assertIsOK(SimUtils.initializeCTRECANCoderSim(verticalElevatorEncoder));

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
        Phoenix6Utils.assertIsOK(SimUtils.initializeCTRECANCoderSim(horizontalElevatorEncoder));

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
