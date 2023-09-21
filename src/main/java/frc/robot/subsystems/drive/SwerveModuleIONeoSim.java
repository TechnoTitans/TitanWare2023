package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.RevSparkMAXSim;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIONeoSim implements SwerveModuleIO {
    private final TitanSparkMAX driveMotor, turnMotor;
    private final RevSparkMAXSim driveSim, turnSim;
    private final SparkMaxAbsoluteEncoder turnEncoder;
    private final double magnetOffset;

    private final boolean driveInvertedValue, turnInvertedValue;
    private final SparkMaxPIDController driveSparkMaxPID;
    private final SparkMaxPIDController turnSparkMaxPID;

    private final DeltaTime deltaTime;

    public SwerveModuleIONeoSim(
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

        this.turnEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        this.magnetOffset = magnetOffset;

        this.driveSim = new RevSparkMAXSim(
                driveMotor,
                DCMotor.getNEO(1),
//                Constants.Modules.DRIVER_GEAR_RATIO,
                new DCMotorSim(
                        DCMotor.getNeo550(1),
                        Constants.Modules.DRIVER_GEAR_RATIO,
                        Constants.Modules.DRIVE_WHEEL_MOMENT_OF_INERTIA
                )
        );

        this.turnSim = new RevSparkMAXSim(
                turnMotor,
                DCMotor.getNEO(1),
//                Constants.Modules.TURNER_GEAR_RATIO,
                new DCMotorSim(
                        DCMotor.getNeo550(1),
                        Constants.Modules.TURNER_GEAR_RATIO,
                        Constants.Modules.TURN_WHEEL_MOMENT_OF_INERTIA
                )
        );
        //todo change
//        this.turnSim.attachRemoteSensor(turnEncoder);

        this.deltaTime = new DeltaTime();

        config();
    }

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
        driveMotor.getEncoder().setPositionConversionFactor(Constants.Modules.DRIVER_GEAR_RATIO);
        driveMotor.getEncoder().setVelocityConversionFactor(Constants.Modules.DRIVER_GEAR_RATIO);
//        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;

        turnMotor.restoreFactoryDefaults();
        turnSparkMaxPID.setP(1);
        turnSparkMaxPID.setD(0);
//        turnSparkMaxPID.setOutputRange(-0.5, 0.5);
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
    public void periodic() {
        final double dtSeconds = deltaTime.get();
        driveSim.update(dtSeconds);
        turnSim.update(dtSeconds);
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
        Logger.getInstance().recordOutput("desiredTurnerRotations", desiredTurnerRotations);
        Logger.getInstance().recordOutput("currentTurnerRotations", turnEncoder.getPosition());
        turnSparkMaxPID.setReference(desiredTurnerRotations, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
    }
}
