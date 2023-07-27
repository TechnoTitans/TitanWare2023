package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.sim.CTREPhoenix6TalonFXSim;
import frc.robot.utils.sim.SimUtils;

public class SwerveModuleIOImpl implements SwerveModuleIO {
    private final TalonFX driveMotor, turnMotor;
    private final CTREPhoenix6TalonFXSim driveSim, turnSim;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final InvertedValue driveInvertedValue, turnInvertedValue;
    private final TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonFXConfiguration = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VelocityVoltage velocityVoltage;

    private final PositionVoltage positionVoltage;

    private final boolean isReal;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    public SwerveModuleIOImpl(
            final TalonFX driveMotor,
            final TalonFX turnMotor,
            final CANcoder turnEncoder,
            final InvertedValue driveInvertedValue,
            final InvertedValue turnInvertedValue,
            final double magnetOffset
    ) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.driveInvertedValue = driveInvertedValue;
        this.driveSim = new CTREPhoenix6TalonFXSim(
                driveMotor,
                Constants.Modules.DRIVER_GEAR_RATIO,
                new DCMotorSim(
                        DCMotor.getFalcon500(1),
                        Constants.Modules.DRIVER_GEAR_RATIO,
                        Constants.Modules.DRIVE_WHEEL_MOMENT_OF_INERTIA
                )
        );

        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        this.turnInvertedValue = turnInvertedValue;
        this.turnSim = new CTREPhoenix6TalonFXSim(
                turnMotor,
                Constants.Modules.TURNER_GEAR_RATIO,
                new DCMotorSim(
                        DCMotor.getFalcon500(1),
                        Constants.Modules.TURNER_GEAR_RATIO,
                        Constants.Modules.TURN_WHEEL_MOMENT_OF_INERTIA
                )
        );
        this.turnSim.attachRemoteSensor(turnEncoder);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.velocityVoltage = new VelocityVoltage(0);
        this.positionVoltage = new PositionVoltage(0);

        this.isReal = Constants.CURRENT_MODE == Constants.RobotMode.REAL;

        config();
    }

    @Override
    public void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        driveTalonFXConfiguration.Slot0 = isReal
                ? Constants.Modules.DRIVE_MOTOR_CONSTANTS
                : Constants.Sim.DRIVE_MOTOR_CONSTANTS;
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveTalonFXConfiguration.MotorOutput.Inverted = driveInvertedValue;

        SimUtils.setCTRETalonFXSimStateMotorInverted(driveMotor, driveInvertedValue);
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        turnTalonFXConfiguration.Slot0 = isReal
                ? Constants.Modules.TURN_MOTOR_CONSTANTS
                : Constants.Sim.TURN_MOTOR_CONSTANTS;
        turnTalonFXConfiguration.Voltage.PeakForwardVoltage = 6;
        turnTalonFXConfiguration.Voltage.PeakReverseVoltage = -6;
        turnTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnTalonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnTalonFXConfiguration.MotorOutput.Inverted = turnInvertedValue;

        SimUtils.setCTRETalonFXSimStateMotorInverted(turnMotor, turnInvertedValue);
        turnMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }

    @Override
    public void periodic() {
        if (!isReal) {
            driveSim.update(Constants.LOOP_PERIOD_SECONDS);
            turnSim.update(Constants.LOOP_PERIOD_SECONDS);
        }
    }

    @Override
    public void updateInputs(final SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.lastDesiredStates = LogUtils.toDoubleArray(getLastDesiredState());

        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = getDriveVelocity();
        inputs.driveDesiredVelocityRotsPerSec = compute_desired_driver_velocity(getLastDesiredState());
        inputs.driveCurrentAmps = driveMotor.getTorqueCurrent().refresh().getValue();
        inputs.driveTempCelsius = driveMotor.getDeviceTemp().refresh().getValue();

        inputs.turnAbsolutePositionRots = getAngle().getRotations();
        inputs.turnDesiredAbsolutePositionRots = compute_desired_turner_rotations(getLastDesiredState());
        inputs.turnVelocityRotsPerSec = turnEncoder.getVelocity().refresh().getValue();
        inputs.turnCurrentAmps = turnMotor.getTorqueCurrent().refresh().getValue();
        inputs.turnTempCelsius = turnMotor.getDeviceTemp().refresh().getValue();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
                Phoenix6Utils.latencyCompensateIfSignalIsGood(
                        turnEncoder.getAbsolutePosition(), turnEncoder.getVelocity()
                )
        );
    }

    public double getDrivePosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(driveMotor.getPosition(), driveMotor.getVelocity());
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().refresh().getValue();
    }

    /**
     * Compute the desired drive motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor velocity given wheel velocity (rps)
     * @param wantedState the wanted state of the module
     * @return the desired rotor velocity
     * @see SwerveModuleState
     */
    public double compute_desired_driver_velocity(final SwerveModuleState wantedState) {
        return wantedState.speedMetersPerSecond / Constants.Modules.WHEEL_CIRCUMFERENCE;
    }

    /**
     * Compute the desired turn motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor position given wheel rotational position (rots)
     * @param wantedState the wanted state of the module
     * @return the desired rotor position
     * @see SwerveModuleState
     */
    public double compute_desired_turner_rotations(final SwerveModuleState wantedState) {
        return wantedState.angle.getRotations();
    }

    @Override
    public void setDesiredState(final SwerveModuleState state) {
        final Rotation2d currentWheelRotation = getAngle();
        final SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        final double desired_driver_velocity = compute_desired_driver_velocity(wantedState);
        final double desired_turner_rotations = compute_desired_turner_rotations(wantedState);
        this.lastDesiredState = wantedState;

        if (!isReal && Constants.Sim.USE_VELOCITY_VOLTAGE_IN_SIM) {
            driveMotor.setControl(velocityVoltage.withVelocity(desired_driver_velocity));
        } else {
            driveMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desired_driver_velocity));
        }

        turnMotor.setControl(positionVoltage.withPosition(desired_turner_rotations));
    }

    @Override
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        final StatusCode refreshCode = driveMotor.getConfigurator().refresh(turnTalonFXConfiguration);
        if (!refreshCode.isOK()) {
            // only warn if in real, there seems to be an issue with calling refresh while disabled in sim
            // TODO: investigate this further
            if (isReal) {
                DriverStation.reportWarning(
                        String.format(
                                "Failed to set NeutralMode on TalonFX %s (%s)",
                                driveMotor.getDeviceID(),
                                driveMotor.getCANBus()
                        ), false
                );
            }

            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;

        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
