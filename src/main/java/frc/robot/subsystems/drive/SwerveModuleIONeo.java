package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utils.rev.RevUtils;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class SwerveModuleIONeo implements SwerveModuleIO {
    private final TitanSparkMAX driveMotor, turnMotor;
    private final SparkMaxAbsoluteEncoder turnEncoder;
    private final double magnetOffset;

    private final boolean driveInvertedValue, turnInvertedValue;
    private final SparkMaxPIDController driveSparkMaxPID;
    private final SparkMaxPIDController turnSparkMaxPID;

    public SwerveModuleIONeo(
            final TitanSparkMAX driveMotor,
            final TitanSparkMAX turnMotor,
            final InvertedValue driveInvertedValue,
            final InvertedValue turnInvertedValue,
            final double magnetOffset
    ) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.driveSparkMaxPID = driveMotor.getPIDController();
        this.turnSparkMaxPID = turnMotor.getPIDController();

        this.driveInvertedValue = RevUtils.convertInvertedModeToBoolean(driveInvertedValue);
        this.turnInvertedValue = RevUtils.convertInvertedModeToBoolean(turnInvertedValue);

        this.turnEncoder = driveMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.magnetOffset = magnetOffset;

        config();
    }

    @Override
    public void config() {
        //todo: no idea if this is right
        turnEncoder.setZeroOffset(-magnetOffset);

        driveMotor.restoreFactoryDefaults();
        driveSparkMaxPID.setP(Constants.Modules.DRIVE_MOTOR_CONSTANTS.kP);
        driveSparkMaxPID.setD(Constants.Modules.DRIVE_MOTOR_CONSTANTS.kD);
        driveMotor.setClosedLoopRampRate(0.2);
        driveMotor.disableVoltageCompensation();
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveMotor.setInverted(driveInvertedValue);
        driveMotor.setSmartCurrentLimit(60);
        //todo: no idea if this is right
        driveMotor.getEncoder().setPositionConversionFactor(1/Constants.Modules.DRIVER_GEAR_RATIO);
        driveMotor.getEncoder().setVelocityConversionFactor(1/Constants.Modules.DRIVER_GEAR_RATIO);
//        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;

        turnMotor.restoreFactoryDefaults();
        turnSparkMaxPID.setP(Constants.Modules.TURN_MOTOR_CONSTANTS.kP);
        turnSparkMaxPID.setD(Constants.Modules.TURN_MOTOR_CONSTANTS.kD);
        turnSparkMaxPID.setOutputRange(-0.5, 0.5);
        turnSparkMaxPID.setPositionPIDWrappingEnabled(true);
        turnSparkMaxPID.setPositionPIDWrappingMinInput(0);
        turnSparkMaxPID.setPositionPIDWrappingMaxInput(1);
        turnSparkMaxPID.setFeedbackDevice(turnEncoder);
        turnMotor.disableVoltageCompensation();
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        turnMotor.setInverted(turnInvertedValue);
//        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
//        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
    }

    @Override
    public void updateInputs(final SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = getDriveVelocity();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();

        inputs.turnAbsolutePositionRots = getAngle().getRotations();
        inputs.turnVelocityRotsPerSec = turnEncoder.getVelocity();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
        inputs.turnTempCelsius = turnMotor.getMotorTemperature();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition());
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        driveSparkMaxPID.setReference(desiredDriverVelocity, CANSparkMax.ControlType.kVelocity);
        turnSparkMaxPID.setReference(desiredTurnerRotations, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        driveMotor.setIdleMode(RevUtils.convertNeutralModeToIdleMode(neutralMode));
    }
}
