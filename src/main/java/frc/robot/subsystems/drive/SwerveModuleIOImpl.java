package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.utils.sim.CTREPhoenix6TalonFXSim;
import frc.robot.utils.sim.SimUtils;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOImpl implements SwerveModuleIO {
    private final TalonFX driveMotor, turnMotor;
    private final CTREPhoenix6TalonFXSim driveSim, turnSim;
    private final CANcoder turnEncoder;
    private final double magnetOffset;
    private final InvertedValue driveInvertedValue, turnInvertedValue;

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
    public boolean isReal() {
        return isReal;
    }

    @Override
    public void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;

        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        final TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.Slot0 = Constants.Modules.DRIVE_MOTOR_CONSTANTS;
        driverConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        driverConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driverConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driverConfig.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driverConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driverConfig.MotorOutput.Inverted = driveInvertedValue;

        SimUtils.setCTRETalonFXSimStateMotorInverted(driveMotor, driveInvertedValue);
        driveMotor.getConfigurator().apply(driverConfig);

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Slot0 = Constants.Modules.TURN_MOTOR_CONSTANTS;
        turnerConfig.Voltage.PeakForwardVoltage = 6;
        turnerConfig.Voltage.PeakReverseVoltage = -6;
        turnerConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnerConfig.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnerConfig.MotorOutput.Inverted = turnInvertedValue;

        SimUtils.setCTRETalonFXSimStateMotorInverted(turnMotor, turnInvertedValue);
        turnMotor.getConfigurator().apply(turnerConfig);
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
        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = getDriveVelocity();
        inputs.driveDesiredVelocityRotsPerSec = compute_desired_driver_velocity(getLastDesiredState());
        inputs.driveCurrentAmps = driveMotor.getTorqueCurrent().refresh().getValue();
        inputs.driveTempCelsius = driveMotor.getDeviceTemp().refresh().getValue();

        inputs.turnAbsolutePositionRots = getAngle().getRotations();
        inputs.turnDesiredAbsolutePositionRotsPerSec = compute_desired_turner_rotations(getLastDesiredState());
        inputs.turnVelocityRotsPerSec = turnEncoder.getVelocity().refresh().getValue();
        inputs.turnCurrentAmps = turnMotor.getTorqueCurrent().refresh().getValue();
        inputs.turnTempCelsius = turnMotor.getDeviceTemp().refresh().getValue();
    }

    @Override
    public Rotation2d getAngle() {
        final double compensatedValue = BaseStatusSignal.getLatencyCompensatedValue(
                turnEncoder.getAbsolutePosition().refresh(),
                turnEncoder.getVelocity().refresh()
        );

        Logger.getInstance().recordOutput("AngleModule_" + driveMotor.getDeviceID(), compensatedValue);

        return Rotation2d.fromRotations(compensatedValue);
    }

    @Override
    public double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(
                driveMotor.getPosition().refresh(),
                driveMotor.getVelocity().refresh()
        );
    }

    @Override
    public double getDriveVelocity() {
        return driveMotor.getVelocity().refresh().getValue();
    }

    @Override
    public SwerveModuleState getState() {
        // #1 max sell of the year
        return new SwerveModuleState(
                getDriveVelocity() * Constants.Modules.WHEEL_CIRCUMFERENCE,
                getAngle()
        );
    }

    @Override
    public SwerveModulePosition getPosition() {
        // #1 max sell of the year
        return new SwerveModulePosition(
                getDrivePosition() * Constants.Modules.WHEEL_CIRCUMFERENCE,
                getAngle()
        );
    }

    @Override
    public double compute_desired_driver_velocity(final SwerveModuleState wantedState) {
        return wantedState.speedMetersPerSecond / Constants.Modules.WHEEL_CIRCUMFERENCE;
    }

    @Override
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

    @Override
    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }
}
