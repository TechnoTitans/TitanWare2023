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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.TitanBoard;

public class SwerveModule {
    private final TalonFX driveMotor, turnMotor;
    private final CANcoder turnEncoder;
    private final double magnetOffset;
    private final InvertedValue driveInvertedValue, turnInvertedValue;

    private final VelocityVoltage velocityVoltage;
    private final PositionVoltage positionVoltage;

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

        this.velocityVoltage = new VelocityVoltage(0);
        this.positionVoltage = new PositionVoltage(0);

        config();
    }

    private void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        final TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.Slot0 = Constants.Modules.DRIVE_MOTOR_CONSTANTS;
        driverConfig.CurrentLimits.StatorCurrentLimit = 80;
        driverConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driverConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
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

        driveMotor.setControl(velocityVoltage.withVelocity(desired_driver_velocity));
        turnMotor.setControl(positionVoltage.withPosition(desired_turner_rotations));

        if (turnMotor.getDeviceID() == 2) {
            SmartDashboard.putNumber("FL drive desired speed", Math.abs(desired_driver_velocity));
            SmartDashboard.putNumber("FL drive current speed", Math.abs(driveMotor.getVelocity().refresh().getValue()));
        }
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
