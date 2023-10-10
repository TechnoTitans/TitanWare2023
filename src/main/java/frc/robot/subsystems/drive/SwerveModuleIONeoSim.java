package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.feedback.SimSparkMaxAbsoluteEncoder;
import frc.robot.utils.sim.motors.RevSparkMAXSim;
import frc.robot.wrappers.control.Slot0Configs;
import frc.robot.wrappers.motors.TitanSparkMAX;

public class SwerveModuleIONeoSim implements SwerveModuleIO {
    private final TitanSparkMAX driveMotor;
    private final TitanSparkMAX turnMotor;
    private final RevSparkMAXSim driveSim;
    private final RevSparkMAXSim turnSim;
    private final double magnetOffset;

    private final InvertedValue driveInvertedValue;
    private final InvertedValue turnInvertedValue;

    private final RelativeEncoder driveRelativeEncoder;
    private final SparkMaxPIDController driveSparkMaxPID;

    private final Slot0Configs turnMotorGains = new Slot0Configs(0.1, 0, 0, 0);
    private final SparkMaxPIDController turnSparkMaxControllerPID;
    private final PIDController turnSparkMaxRoborioPID;
    private final SimpleMotorFeedforward turnMotorFeedforward;
    private final SimSparkMaxAbsoluteEncoder turnAbsoluteEncoder;

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
        this.magnetOffset = magnetOffset;

        this.driveInvertedValue = driveInvertedValue;
        this.turnInvertedValue = turnInvertedValue;

        this.driveSparkMaxPID = driveMotor.getPIDController();
        this.turnSparkMaxControllerPID = turnMotor.getPIDController();

        this.driveRelativeEncoder = driveMotor.getEncoder();
        this.turnAbsoluteEncoder = new SimSparkMaxAbsoluteEncoder(turnMotor, SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        final DCMotor driveDCMotor = DCMotor.getNEO(1);
        this.driveSim = new RevSparkMAXSim(
                driveMotor,
                driveDCMotor,
                new DCMotorSim(
                        driveDCMotor,
                        Constants.Modules.DRIVER_GEAR_RATIO,
                        Constants.Modules.DRIVE_WHEEL_MOMENT_OF_INERTIA
                )
        );

        final DCMotor turnDCMotor = DCMotor.getNEO(1);
        this.turnSim = new RevSparkMAXSim(
                turnMotor,
                turnDCMotor,
                new DCMotorSim(
                        turnDCMotor,
                        Constants.Modules.TURNER_GEAR_RATIO,
                        Constants.Modules.TURN_WHEEL_MOMENT_OF_INERTIA
                )
        );
        this.turnSim.attachFeedbackSensor(turnAbsoluteEncoder);

        this.deltaTime = new DeltaTime();

        this.turnSparkMaxRoborioPID = new PIDController(turnMotorGains.kP, turnMotorGains.kI, turnMotorGains.kD);
        this.turnSparkMaxRoborioPID.enableContinuousInput(-1, 1);

        this.turnMotorFeedforward = new SimpleMotorFeedforward(turnMotorGains.kS, turnMotorGains.kV);
    }

    @Override
    @SuppressWarnings("DuplicatedCode")
    public void config() {
        // TODO: does any of this configuration work correctly? we need to change a lot of this, probably
        // Drive SparkMAX & Motor
        driveMotor.restoreFactoryDefaults();

        // TODO: what does voltage compensation do??
        driveMotor.enableVoltageCompensation(Constants.PDH.BATTERY_NOMINAL_VOLTAGE);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveMotor.setInverted(RevUtils.invertedValueToBoolean(driveInvertedValue));
        driveMotor.setSmartCurrentLimit(60);

        // TODO: we should probably stop using Phoenix 6 stuff within rev configurations
        //  it'll probably be better for us to just make a PID gains wrapper for rev instead
        //  particularly because kV doesn't even exist for rev
        final Slot0Configs driveMotorGains = new Slot0Configs(0.1, 0, 0, 0);
        driveSparkMaxPID.setP(driveMotorGains.kP);
        driveSparkMaxPID.setI(driveMotorGains.kI);
        driveSparkMaxPID.setD(driveMotorGains.kD);
        driveSparkMaxPID.setFF(driveMotorGains.kS);

        driveRelativeEncoder.setPosition(0);
        driveSparkMaxPID.setFeedbackDevice(driveRelativeEncoder);

        // Turn SparkMAX & Motor
        turnMotor.restoreFactoryDefaults();

        // TODO: what does voltage compensation do??
        turnMotor.enableVoltageCompensation(Constants.PDH.BATTERY_NOMINAL_VOLTAGE);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        turnMotor.setInverted(RevUtils.invertedValueToBoolean(turnInvertedValue));
        turnMotor.setSmartCurrentLimit(35);

        turnSparkMaxControllerPID.setP(turnMotorGains.kP);
        turnSparkMaxControllerPID.setI(turnMotorGains.kI);
        turnSparkMaxControllerPID.setD(turnMotorGains.kD);
        turnSparkMaxControllerPID.setFF(turnMotorGains.kS);

        // TODO: does this work as expected?
        turnAbsoluteEncoder.setZeroOffset(magnetOffset);
        turnSparkMaxControllerPID.setFeedbackDevice(turnMotor.getEncoder());

        turnSparkMaxControllerPID.setPositionPIDWrappingEnabled(true);
        // TODO: these should probably be constants somewhere
        turnSparkMaxControllerPID.setPositionPIDWrappingMinInput(-1);
        turnSparkMaxControllerPID.setPositionPIDWrappingMaxInput(1);

        // Burn settings to flash on Drive and Turn
        // TODO: pretty sure this needs a delay...not sure how to cleanly do that here
        //  also, the previous configs are sometimes applied multiple times by some teams
        driveMotor.burnFlash();
        turnMotor.burnFlash();
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
        // TODO: getOutputCurrent doesn't even work in sim... figure out a way to calculate a semi-correct current
        //  for NEOs in sim
        inputs.driveTorqueCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveStatorCurrentAmps = 0;
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();

        inputs.turnAbsolutePositionRots = getRawAngle();
        inputs.turnVelocityRotsPerSec = turnAbsoluteEncoder.getVelocity();
        // TODO: getOutputCurrent doesn't even work in sim... figure out a way to calculate a semi-correct current
        //  for NEOs in sim
        inputs.turnTorqueCurrentAmps = turnMotor.getOutputCurrent();
        inputs.turnStatorCurrentAmps = 0;
        inputs.turnTempCelsius = turnMotor.getMotorTemperature();
    }

    /**
     * Get the rotation of the wheel (mechanism)
     * @return the current rotation of the wheel, in absolute rotations
     */
    private double getRawAngle() {
        return turnAbsoluteEncoder.getPosition();
    }

    /**
     * Get the drive motor position (mechanism)
     * @return the rotor position, in rots (rotations)
     */
    public double getDrivePosition() {
        return driveRelativeEncoder.getPosition() / Constants.Modules.DRIVER_GEAR_RATIO;
    }

    /**
     * Get the drive motor velocity (mechanism)
     * @return the rotor velocity, in rots/sec (rps)
     */
    public double getDriveVelocity() {
        return RevUtils.rotationsPerMinuteToRotationsPerSecond(driveRelativeEncoder.getVelocity())
                / Constants.Modules.DRIVER_GEAR_RATIO;
    }

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        driveSparkMaxPID.setReference(
                RevUtils.rotationsPerSecondToRotationsPerMinute(desiredDriverVelocity)
                        * Constants.Modules.DRIVER_GEAR_RATIO,
                CANSparkMax.ControlType.kVelocity
        );

        if (SimConstants.Rev.SIM_USE_ROBORIO_PID_FOR_POSITION) {
            turnSparkMaxControllerPID.setReference(
                    turnSparkMaxRoborioPID.calculate(getRawAngle(), desiredTurnerRotations)
                            + turnMotorFeedforward.calculate(0),
                    CANSparkMax.ControlType.kVoltage
            );
        } else {
            turnSparkMaxControllerPID.setReference(desiredTurnerRotations, CANSparkMax.ControlType.kPosition);
        }
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        // TODO: maybe we should use our own NeutralMode enum that can be converted between ctre and rev?
        driveMotor.setIdleMode(RevUtils.neutralModeToIdleMode(neutralMode));
    }
}
