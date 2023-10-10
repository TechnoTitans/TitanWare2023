package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.utils.ctre.Phoenix6Utils;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final InvertedValue driveInvertedValue;
    private final InvertedValue turnInvertedValue;
    private final TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonFXConfiguration = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final PositionVoltage positionVoltage;

    public SwerveModuleIOFalcon(
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

        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        this.turnInvertedValue = turnInvertedValue;

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.positionVoltage = new PositionVoltage(0);
    }

    @Override
    public void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        driveTalonFXConfiguration.Slot0 = Constants.Modules.Falcon.DRIVE_MOTOR_CONSTANTS;
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveTalonFXConfiguration.MotorOutput.Inverted = driveInvertedValue;
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        turnTalonFXConfiguration.Slot0 = Constants.Modules.Falcon.TURN_MOTOR_CONSTANTS;
        turnTalonFXConfiguration.Voltage.PeakForwardVoltage = 6;
        turnTalonFXConfiguration.Voltage.PeakReverseVoltage = -6;
        turnTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnTalonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnTalonFXConfiguration.MotorOutput.Inverted = turnInvertedValue;
        turnMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }

    @Override
    @SuppressWarnings("DuplicatedCode")
    public void updateInputs(final SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = getDriveVelocity();
        inputs.driveTorqueCurrentAmps = driveMotor.getTorqueCurrent().refresh().getValue();
        inputs.driveStatorCurrentAmps = driveMotor.getStatorCurrent().refresh().getValue();
        inputs.driveTempCelsius = driveMotor.getDeviceTemp().refresh().getValue();

        inputs.turnAbsolutePositionRots = getAngle().getRotations();
        inputs.turnVelocityRotsPerSec = turnEncoder.getVelocity().refresh().getValue();
        inputs.turnTorqueCurrentAmps = turnMotor.getTorqueCurrent().refresh().getValue();
        inputs.turnStatorCurrentAmps = turnMotor.getStatorCurrent().refresh().getValue();
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

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        driveMotor.setControl(velocityTorqueCurrentFOC.withVelocity(desiredDriverVelocity));
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        final StatusCode refreshCode = driveMotor.getConfigurator().refresh(turnTalonFXConfiguration);
        if (!refreshCode.isOK()) {
            DriverStation.reportWarning(
                    String.format(
                            "Failed to set NeutralMode on TalonFX %s (%s)",
                            driveMotor.getDeviceID(),
                            driveMotor.getCANBus()
                    ), false
            );
            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;
        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
