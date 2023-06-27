package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX driveMotor, turnMotor;
    private final CANcoder turnEncoder;
    private final double magnetOffset;
    private final InvertedValue driveInvertedValue, turnInvertedValue;
    private final VelocityTorqueCurrentFOC torqueVelocity;
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

        this.torqueVelocity = new VelocityTorqueCurrentFOC(0);
        this.positionVelocity = new PositionVoltage(0);

        config();
    }

    private void config() {
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
        driveMotor.getConfigurator().apply(driverConfig);

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Slot0 = Constants.Modules.TURN_MOTOR_CONSTANTS;
//        turnerConfig.Voltage.PeakForwardVoltage = 6;
//        turnerConfig.Voltage.PeakReverseVoltage = -6;
        turnerConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnerConfig.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnerConfig.MotorOutput.Inverted = turnInvertedValue;
        turnMotor.getConfigurator().apply(turnerConfig);
    }

    private Rotation2d getAngle() {
        final double compensatedValue = BaseStatusSignal.getLatencyCompensatedValue(
                turnEncoder.getAbsolutePosition().refresh(),
                turnEncoder.getVelocity().refresh()
        );

        return Rotation2d.fromRotations(compensatedValue);
    }

    private double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(
                driveMotor.getPosition().refresh(),
                driveMotor.getVelocity().refresh()
        );
    }

    private double getDriveVelocity() {
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

        driveMotor.setControl(torqueVelocity.withVelocity(desired_driver_velocity));
        turnMotor.setControl(positionVelocity.withPosition(desired_turner_rotations));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
