package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.utils.Enums;
import frc.robot.utils.MathUtils;
import frc.robot.utils.sim.CTREPhoenix5TalonSRXSim;
import frc.robot.utils.sim.SimUtils;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;

public class ClawIOSim implements ClawIO {
    private final TitanSRX clawMainWheelBag, clawFollowerWheelBag;
    private final CTREPhoenix5TalonSRXSim clawMainWheelSim, clawFollowerWheelSim;
    private final TitanSRX clawOpenCloseMotor;
    private final CTREPhoenix5TalonSRXSim clawOpenCloseSim;
    private final CANCoder clawOpenCloseEncoder;
    private final CANcoder clawTiltEncoder;
    private final TitanMAX clawTiltNeo;
    private final DigitalInput clawTiltLimitSwitch;

    private final ProfiledPIDController tiltPID;

    private Enums.ClawState desiredState;
    private Enums.ClawState currentState;

    private ControlMode openCloseControlMode;
    private Enums.ClawControlMode clawControlMode;

    //Claw Intake Wheel Speed
    private double desiredIntakeWheelsPercentOutput = 0;
    //Claw Tilt Rotations
    private double desiredTiltPositionRots = 0;
    //Claw Open Close Ticks
    private double desiredOpenClosePositionRots = 0;

    //Simulation
    private boolean clawTiltLimitSwitchState = false;
    private double currentTiltPositionRots = 0;
    private double currentOpenClosePositionRots = 0;

    public ClawIOSim(
            final TitanSRX clawMainWheelBag,
            final TitanSRX clawFollowerWheelBag,
            final TitanSRX clawOpenCloseMotor,
            final CANCoder clawOpenCloseEncoder,
            final TitanMAX clawTiltNeo,
            final CANcoder clawTiltEncoder,
            final DigitalInput clawTiltLimitSwitch
    ) {
        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;

        //TODO: get real MOI for compliant wheels
        this.clawMainWheelSim = new CTREPhoenix5TalonSRXSim(
                clawMainWheelBag,
                new DCMotorSim(
                    DCMotor.getBag(1),
                    1,
                    1
                )
        );

        //TODO: get real MOI for compliant wheels
        this.clawFollowerWheelSim = new CTREPhoenix5TalonSRXSim(
                clawFollowerWheelBag,
                new DCMotorSim(
                        DCMotor.getBag(1),
                        1,
                        1
                )
        );

        this.clawTiltNeo = clawTiltNeo;
        this.clawTiltEncoder = clawTiltEncoder;
        this.clawTiltLimitSwitch = clawTiltLimitSwitch;

        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        //TODO: get real MOI for open close
        this.clawOpenCloseSim = new CTREPhoenix5TalonSRXSim(
                clawOpenCloseMotor,
                new DCMotorSim(
                        DCMotor.getBag(1),
                        1,
                        1
                )
        );

        //TODO: tune pid
        this.tiltPID = new ProfiledPIDController(
                3, 0, 0,
                new TrapezoidProfile.Constraints(3, 5)
        );

        config();
        setDesiredState(Enums.ClawState.CLAW_STANDBY);
    }

    @Override
    public void periodic() {
        clawMainWheelSim.update(Constants.LOOP_PERIOD_SECONDS);
        clawFollowerWheelSim.update(Constants.LOOP_PERIOD_SECONDS);
        clawOpenCloseSim.update(Constants.LOOP_PERIOD_SECONDS);

        //TODO: do tilt sim
        if (clawTiltLimitSwitchState && clawControlMode == Enums.ClawControlMode.DUTY_CYCLE) {
            clawTiltEncoder.setPosition(0);
            desiredTiltPositionRots = 0.1;
        }

        clawMainWheelBag.set(ControlMode.PercentOutput, desiredIntakeWheelsPercentOutput);
        clawOpenCloseMotor.set(openCloseControlMode, desiredOpenClosePositionRots);

        //TODO: do tilt sim
        switch (clawControlMode) {
            case POSITION -> clawTiltNeo.set(
                    tiltPID.calculate(
                            clawTiltEncoder.getAbsolutePosition().refresh().getValue(), desiredTiltPositionRots
                    )
            );
            case DUTY_CYCLE -> clawTiltNeo.set(desiredTiltPositionRots);
        }
    }

    @Override
    public void updateInputs(final ClawIO.ClawIOInputs inputs) {
        isAtDesiredState();

        inputs.currentTiltEncoderPositionRots = currentTiltPositionRots;
        inputs.desiredTiltPositionRots = desiredTiltPositionRots;
        inputs.currentOpenCloseEncoderPositionRots = currentOpenClosePositionRots;
        inputs.desiredOpenClosePositionRots = desiredOpenClosePositionRots;
        inputs.desiredIntakeWheelsPercentOutput = desiredIntakeWheelsPercentOutput;
        inputs.openCloseCurrentAmps = clawOpenCloseMotor.getCurrent();
        inputs.openCloseControlMode = openCloseControlMode.toString();
        inputs.tiltClawControlMode = clawControlMode.toString();
        inputs.desiredState = desiredState.toString();
        inputs.currentState = currentState.toString();
        inputs.tiltLimitSwitch = clawTiltLimitSwitch.get();
    }

    public void config() {
        // Bag Motors
        clawMainWheelBag.configFactoryDefault();
        clawFollowerWheelBag.configFactoryDefault();
        clawFollowerWheelBag.follow(clawMainWheelBag);

        // Claw Open Close Encoder
        final CANCoderConfiguration clawOpenCloseEncoderConfig = new CANCoderConfiguration();
        clawOpenCloseEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        clawOpenCloseEncoderConfig.unitString = Constants.CTRE.PHOENIX_5_CANCODER_UNIT_STRING_ROTS;
        clawOpenCloseEncoderConfig.sensorDirection = false;
        clawOpenCloseEncoderConfig.sensorCoefficient = Constants.CTRE.PHOENIX_5_CANCODER_SENSOR_COEFFICIENT_ROTS;
        clawOpenCloseEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        clawOpenCloseEncoderConfig.magnetOffsetDegrees = -81.387;

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
        clawOpenCloseMotor.brake();

        // Claw Tilt Neo
        clawTiltNeo.brake();
        clawTiltNeo.currentLimit(25);

        // Claw Tilt Encoder
        final SensorDirectionValue clawTiltEncoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        final CANcoderConfiguration clawTiltEncoderConfig = new CANcoderConfiguration();
        clawTiltEncoderConfig.MagnetSensor.SensorDirection = clawTiltEncoderSensorDirection;
        clawTiltEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        clawTiltEncoderConfig.MagnetSensor.MagnetOffset = -0.17;

        SimUtils.setCTRECANCoderSimStateSensorDirection(clawTiltEncoder, clawTiltEncoderSensorDirection);

        clawTiltEncoder.getConfigurator().apply(clawTiltEncoderConfig);
    }

    public void setDesiredState(final Enums.ClawState state) {
        desiredState = state;

        desiredIntakeWheelsPercentOutput = state.getIntakeWheelsPercentOutput();
        clawControlMode = state.getClawControlMode();
        desiredTiltPositionRots = state.getTiltPositionRots();
        openCloseControlMode = state.getOpenCloseControlMode();
        desiredOpenClosePositionRots = state.getOpenCloseRots();
    }

    public boolean isAtDesiredState() {
        if (currentState == desiredState) {
            return true;
        } else {
            //TODO: figure out what units are where
            final boolean isAtDesired =
                    MathUtils.withinRange(
                            clawOpenCloseMotor.getSelectedSensorPosition(),
                            desiredOpenClosePositionRots,
                            5
                    ) && MathUtils.withinRange(
                            clawTiltEncoder.getAbsolutePosition().refresh().getValue(),
                            desiredTiltPositionRots,
                            5
                    );

            if (isAtDesired) {
                currentState = desiredState;
            }

            return isAtDesired;
        }
    }

    public Enums.ClawState getDesiredState() {
        return desiredState;
    }

    public Enums.ClawState getCurrentState() {
        return currentState;
    }
}
