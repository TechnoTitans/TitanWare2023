package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.utils.sim.SimUtils;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final TalonFX driveMotor, turnMotor;
    private final DCMotorSim driveSim, turnSim;
    private final InvertedValue driveInvertedValue, turnInvertedValue;

    private final PositionVoltage positionVoltage;

    private final VelocityVoltage driveVoltage;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    public double drivePositionRots = 0.0;
    public double driveVelocityRotsPerSec = 0.0;

    private double turnAbsolutePositionRots = 0.0;
    private double turnVelocityRotsPerSec = 0.0;

    @SuppressWarnings("unused")
    public SwerveModuleIOSim(
            final TalonFX driveMotor,
            final TalonFX turnMotor,
            final CANcoder turnEncoder,
            final InvertedValue driveInvertedValue,
            final InvertedValue turnInvertedValue,
            final double magnetOffset
    ) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.driveSim = new DCMotorSim(
                DCMotor.getFalcon500(1),
                Constants.Modules.DRIVER_GEAR_RATIO,
                Constants.Modules.DRIVE_WHEEL_MOMENT_OF_INERTIA
        );

        this.turnSim = new DCMotorSim(
                DCMotor.getFalcon500(1),
                Constants.Modules.TURNER_GEAR_RATIO,
                Constants.Modules.TURN_WHEEL_MOMENT_OF_INERTIA
        );

        this.driveInvertedValue = driveInvertedValue;
        this.turnInvertedValue = turnInvertedValue;

        this.positionVoltage = new PositionVoltage(0);

        this.driveVoltage = new VelocityVoltage(0);

        config();
    }

    @Override
    public boolean isReal() {
        return false;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.Slot0 = Constants.Sim.DRIVE_MOTOR_CONSTANTS;
        driverConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        driverConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driverConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driverConfig.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driverConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driverConfig.MotorOutput.Inverted = driveInvertedValue;

        SimUtils.setCTRETalonFXSimStateMotorInverted(driveMotor, driveInvertedValue);

        driveMotor.getConfigurator().apply(driverConfig);

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Slot0 = Constants.Sim.TURN_MOTOR_CONSTANTS;
        turnerConfig.Voltage.PeakForwardVoltage = 6;
        turnerConfig.Voltage.PeakReverseVoltage = -6;
        turnerConfig.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnerConfig.MotorOutput.Inverted = turnInvertedValue;

        SimUtils.setCTRETalonFXSimStateMotorInverted(turnMotor, turnInvertedValue);

        turnMotor.getConfigurator().apply(turnerConfig);
    }

    @Override
    public void updateInputs(final SwerveModuleIOInputs inputs) {
        driveSim.setInputVoltage(driveMotor.getSimState().getMotorVoltage());
        turnSim.setInputVoltage(turnMotor.getSimState().getMotorVoltage());

        driveSim.update(Constants.LOOP_PERIOD_SECONDS);
        turnSim.update(Constants.LOOP_PERIOD_SECONDS);

        driveMotor.getSimState().setRawRotorPosition(driveSim.getAngularPositionRotations());
        driveMotor.getSimState().setRotorVelocity(Units.radiansToRotations(driveSim.getAngularVelocityRadPerSec()));
        driveMotor.getSimState().setSupplyVoltage(
                12 - (driveMotor.getSimState().getSupplyCurrent() * Constants.Sim.FALCON_MOTOR_RESISTANCE)
        );

        turnMotor.getSimState().setRawRotorPosition(turnSim.getAngularPositionRotations());
        turnMotor.getSimState().setRotorVelocity(Units.radiansToRotations(turnSim.getAngularVelocityRadPerSec()));
        turnMotor.getSimState().setSupplyVoltage(
                12 - (turnMotor.getSimState().getSupplyCurrent() * Constants.Sim.FALCON_MOTOR_RESISTANCE)
        );

        driveVelocityRotsPerSec = Units.radiansToRotations(driveSim.getAngularVelocityRadPerSec());
        drivePositionRots = driveSim.getAngularPositionRotations();

        turnVelocityRotsPerSec = Units.radiansToRotations(turnSim.getAngularVelocityRadPerSec());
        turnAbsolutePositionRots = turnSim.getAngularPositionRotations();
        //TODO: this sucks pls fix..
        while (turnAbsolutePositionRots < -0.5) {
            turnAbsolutePositionRots += 1;
        }
        while (turnAbsolutePositionRots > 0.5) {
            turnAbsolutePositionRots -= 1;
        }

        inputs.drivePositionRots = drivePositionRots;
        inputs.driveVelocityRotsPerSec = driveVelocityRotsPerSec;
        inputs.driveDesiredVelocityRotsPerSec = compute_desired_driver_velocity(getLastDesiredState());
        inputs.driveCurrentAmps = driveMotor.getSimState().getTorqueCurrent();
        inputs.driveTempCelsius = driveMotor.getDeviceTemp().refresh().getValue();

        inputs.turnAbsolutePositionRots = turnAbsolutePositionRots;
        inputs.turnDesiredAbsolutePositionRotsPerSec = compute_desired_turner_rotations(getLastDesiredState());
        inputs.turnVelocityRotsPerSec = turnVelocityRotsPerSec;
        inputs.turnCurrentAmps = turnMotor.getSimState().getTorqueCurrent();
        inputs.turnTempCelsius = turnMotor.getDeviceTemp().refresh().getValue();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnAbsolutePositionRots);
    }

    @Override
    public double getDrivePosition() {
        return drivePositionRots;
    }

    @Override
    public double getDriveVelocity() {
        return driveVelocityRotsPerSec;
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

        driveMotor.setControl(driveVoltage.withVelocity(desired_driver_velocity));
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
