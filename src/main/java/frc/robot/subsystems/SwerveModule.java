package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor, turnMotor;
    private final CANcoder turnEncoder;
    private final double magnetOffset;
    private final InvertedValue driveInvertedValue, turnInvertedValue;
    private final VelocityVoltage voltageVelocity;
    private final PositionVoltage positionVelocity;

    public SwerveModule(
            final TalonFX driveMotor,
            final TalonFX turnMotor,
            final CANcoder turnEncoder,
            final InvertedValue driveInvertedValue,
            final InvertedValue turnInvertedValue,
            final double magnetOffset
    ) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        this.driveInvertedValue = driveInvertedValue;
        this.turnInvertedValue = turnInvertedValue;

        this.voltageVelocity = new VelocityVoltage(
                0,true, 0, 0, false
        );

        this.positionVelocity = new PositionVoltage(
                0, true, 0, 0, false
        );

        config();
    }

    private void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        final TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.Slot0.kP = 0.00060954;
        driverConfig.Slot0.kD = 0.01;
        driverConfig.Slot0.kS = 0.25655;
        driverConfig.Slot0.kV = 2.9757;
        driverConfig.CurrentLimits.StatorCurrentLimit = 60;
        driverConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driverConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
        driverConfig.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driverConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driverConfig.MotorOutput.Inverted = driveInvertedValue;
        driveMotor.getConfigurator().apply(driverConfig);

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Slot0.kP = 30; //0.47
        turnerConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
        turnerConfig.MotorOutput.PeakReverseDutyCycle = 0.5;
        turnerConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnerConfig.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnerConfig.MotorOutput.Inverted = turnInvertedValue;
        turnMotor.getConfigurator().apply(turnerConfig);
    }

    public Rotation2d getAngle() {
        final double compensatedValue = BaseStatusSignal.getLatencyCompensatedValue(
                turnEncoder.getAbsolutePosition().refresh(),
                turnEncoder.getVelocity().refresh()
        );

        return Rotation2d.fromRotations(compensatedValue);
    }

    public double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(
                driveMotor.getPosition().refresh(),
                driveMotor.getVelocity().refresh()
        );
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().refresh().getValue();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                -getDriveVelocity() * Constants.Modules.WHEEL_CIRCUMFERENCE,
                getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                -getDrivePosition() * Constants.Modules.WHEEL_CIRCUMFERENCE,
                getAngle()
        );
    }

    public void setDesiredState(final SwerveModuleState state) {
        final Rotation2d currentWheelRotation = getAngle();
        final SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        final double desired_driver_velocity = wantedState.speedMetersPerSecond / Constants.Modules.WHEEL_CIRCUMFERENCE;
        final double desired_turner_rotations = wantedState.angle.getRotations();

        driveMotor.setControl(voltageVelocity.withVelocity(desired_driver_velocity));
        turnMotor.setControl(positionVelocity.withPosition(desired_turner_rotations));
    }

    public void percentOutputControl(final double output) {
        driveMotor.set(output);
        turnMotor.set(0);
    }

    public void manualVelocityControl(final double rps) {
        driveMotor.setControl(voltageVelocity.withVelocity(rps));
        turnMotor.set(0);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
