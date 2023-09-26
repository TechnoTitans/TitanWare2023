package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utils.rev.RevUtils;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class SwerveModuleIONeo implements SwerveModuleIO {
    private final TitanSparkMAX driveMotor;
    private final TitanSparkMAX turnMotor;
    private final double magnetOffset;

    private final InvertedValue driveInvertedValue;
    private final InvertedValue turnInvertedValue;

    private final RelativeEncoder driveRelativeEncoder;
    private final SparkMaxPIDController driveSparkMaxPID;

    private final SparkMaxAbsoluteEncoder turnAbsoluteEncoder;
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

        // TODO: maybe stop using InvertedValue from ctre in favor of a custom solution that can be converted into
        //  both a ctre InvertedValue and a rev boolean
        this.driveInvertedValue = driveInvertedValue;
        this.turnInvertedValue = turnInvertedValue;

        this.driveRelativeEncoder = driveMotor.getEncoder();
        // TODO: this AbsoluteEncoder.Type should probably also be a constant
        this.turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        this.magnetOffset = magnetOffset;
    }

    @Override
    @SuppressWarnings("DuplicatedCode")
    public void config() {
        // TODO: does any of this configuration work correctly? we need to change a lot of this, probably
        // Drive SparkMAX & Motor
        driveMotor.restoreFactoryDefaults();

        // TODO: what does voltage compensation do?? do we need to extract the nominal voltage to a constant
        driveMotor.enableVoltageCompensation(12);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveMotor.setInverted(RevUtils.invertedValueToBoolean(driveInvertedValue));
        driveMotor.setSmartCurrentLimit(60);

        // TODO: we should probably stop using Phoenix 6 stuff within rev configurations
        //  it'll probably be better for us to just make a PID gains wrapper for rev instead
        //  particularly because kV doesn't even exist for rev
        final Slot0Configs driveMotorGains = Constants.Modules.Neo.DRIVE_MOTOR_CONSTANTS;
        driveSparkMaxPID.setP(driveMotorGains.kP);
        driveSparkMaxPID.setI(driveMotorGains.kI);
        driveSparkMaxPID.setD(driveMotorGains.kD);
        driveSparkMaxPID.setFF(driveMotorGains.kS);

        driveRelativeEncoder.setPosition(0);
        driveSparkMaxPID.setFeedbackDevice(driveRelativeEncoder);

        // Turn SparkMAX & Motor
        turnMotor.restoreFactoryDefaults();

        // TODO: what does voltage compensation do?? do we need to extract the nominal voltage to a constant
        turnMotor.enableVoltageCompensation(12);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        turnMotor.setInverted(RevUtils.invertedValueToBoolean(turnInvertedValue));
        turnMotor.setSmartCurrentLimit(35);

        turnSparkMaxPID.setFeedbackDevice(turnAbsoluteEncoder);

        final Slot0Configs turnMotorGains = Constants.Modules.Neo.TURN_MOTOR_CONSTANTS;
        turnSparkMaxPID.setP(turnMotorGains.kP);
        turnSparkMaxPID.setI(turnMotorGains.kI);
        turnSparkMaxPID.setD(turnMotorGains.kD);
        turnSparkMaxPID.setFF(turnMotorGains.kS);

        turnSparkMaxPID.setPositionPIDWrappingEnabled(true);
        // TODO: these should probably be constants somewhere
        turnSparkMaxPID.setPositionPIDWrappingMinInput(-1);
        turnSparkMaxPID.setPositionPIDWrappingMaxInput(1);

        // TODO: does this work as expected?
        turnAbsoluteEncoder.setZeroOffset(-magnetOffset);

        // TODO: pretty sure this needs a delay...not sure how to cleanly do that here
        //  also, the previous configs are sometimes applied multiple times by some teams
        driveMotor.burnFlash();
        turnMotor.burnFlash();
    }

    @Override
    public void updateInputs(final SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = getDriveVelocity();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();

        inputs.turnAbsolutePositionRots = getAngle().getRotations();
        inputs.turnVelocityRotsPerSec = turnAbsoluteEncoder.getVelocity();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
        inputs.turnTempCelsius = turnMotor.getMotorTemperature();
    }

    /**
     * Get the rotation of the wheel (mechanism) as a {@link Rotation2d}
     * @return the {@link Rotation2d} describing the current rotation of the wheel
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnAbsoluteEncoder.getPosition());
    }

    /**
     * Get the drive motor position (rotor)
     * @return the rotor position, in rots (rotations)
     */
    public double getDrivePosition() {
        return driveRelativeEncoder.getPosition() / Constants.Modules.DRIVER_GEAR_RATIO;
    }

    /**
     * Get the drive motor velocity (rotor)
     * @return the rotor velocity, in rots/sec (rps)
     */
    public double getDriveVelocity() {
        // TODO: magic number! (converts from RPM to rots/sec, so, not sure what to do about that here)
        //  maybe we can just keep the magic number?
        return driveRelativeEncoder.getVelocity() / 60 / Constants.Modules.DRIVER_GEAR_RATIO;
    }

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        // TODO: magic number again... converts from rots/sec to RPM for rev
        driveSparkMaxPID.setReference(
                desiredDriverVelocity
                        * 60
                        * Constants.Modules.DRIVER_GEAR_RATIO,
                CANSparkMax.ControlType.kVelocity
        );
        turnSparkMaxPID.setReference(desiredTurnerRotations, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        // TODO: maybe we should use our own NeutralMode enum that can be converted between ctre and rev?
        driveMotor.setIdleMode(RevUtils.neutralModeToIdleMode(neutralMode));
    }
}
