package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.utils.logging.LogUtils;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String name;
    private final String logKey;
    private final SwerveModuleIO moduleIO;
    private final SwerveModuleIOInputsAutoLogged inputs;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    public SwerveModule(final SwerveModuleIO moduleIO, final String name) {
        this.name = name;
        this.logKey = String.format("%s/Module_%s", Swerve.logKey, name);

        this.moduleIO = moduleIO;
        this.moduleIO.config();

        this.inputs = new SwerveModuleIOInputsAutoLogged();
    }

    /**
     * Scales a {@link SwerveModuleState} by the cosine of the error between the {@link SwerveModuleState#angle} and
     * the measured angle (wheel rotation) by mutating its {@link SwerveModuleState#speedMetersPerSecond}.
     * <p> This should be called <b>AFTER</b> {@link SwerveModuleState#optimize(SwerveModuleState, Rotation2d)}</p>
     * @param state the {@link SwerveModuleState} to scale (this is mutated!)
     * @param wheelRotation the measured wheel {@link Rotation2d}
     */
    public static void scaleWithErrorCosine(final SwerveModuleState state, final Rotation2d wheelRotation) {
        state.speedMetersPerSecond *= state.angle.minus(wheelRotation).getCos();
    }

    public String getName() { return name; }

    public void periodic() {
        moduleIO.periodic();
        moduleIO.updateInputs(inputs);

        final Logger logger = Logger.getInstance();
        final double modulePeriodicUpdateStart = logger.getRealTimestamp();

        logger.processInputs(logKey, inputs);

        logger.recordOutput(logKey + "/CurrentState", getState());
        logger.recordOutput(logKey + "/LastDesiredState", lastDesiredState);
        logger.recordOutput(
                logKey + "/DriveDesiredVelocityRotsPerSec",
                computeDesiredDriverVelocity(lastDesiredState)
        );

        logger.recordOutput(
                logKey + "/TurnDesiredAbsolutePositionRots",
                computeDesiredTurnerRotations(lastDesiredState)
        );

        logger.recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getInstance().getRealTimestamp() - modulePeriodicUpdateStart)
        );
    }

    /**
     * Get the current draw (stator current) of the {@link SwerveModule} (drive & turn motors)
     * @return the total current draw of the {@link SwerveModule}, in amps
     */
    public double getCurrentDrawAmps() { return inputs.driveStatorCurrentAmps + inputs.driveStatorCurrentAmps; }

    /**
     * Get a {@link Rotation2d} of the current absolute turn position (computed from encoder rotations)
     * @return the absolute turn position as a {@link Rotation2d}
     * @see Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.turnAbsolutePositionRots);
    }

    /**
     * Get the current relative drive wheel (mechanism) position in rotations
     * @return drive wheel position (rots)
     */
    public double getDrivePosition() {
        return inputs.drivePositionRots;
    }

    /**
     * Get the current drive wheel (mechanism) velocity in rotations/sec
     * @return drive wheel velocity (rps)
     */
    public double getDriveVelocity() {
        return inputs.driveVelocityRotsPerSec;
    }

    /**
     * Get the current module observed {@link SwerveModuleState} (velocity, angle)
     * Velocity is wheel linear velocity, angle is wheel absolute position
     * @return the module's current state as a {@link SwerveModuleState}
     * @see SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity() * Constants.Modules.WHEEL_CIRCUMFERENCE,
                getAngle()
        );
    }

    /**
     * Get the current module observed {@link SwerveModulePosition} (position, angle)
     * Velocity is wheel linear position, angle is wheel absolute position
     * @return the module's current position as a {@link SwerveModulePosition}
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition() * Constants.Modules.WHEEL_CIRCUMFERENCE,
                getAngle()
        );
    }

    /**
     * Compute the desired drive motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor velocity given wheel velocity (rps)
     * @param wantedState the wanted state of the module
     * @return the desired rotor velocity
     * @see SwerveModuleState
     */
    public double computeDesiredDriverVelocity(final SwerveModuleState wantedState) {
        return wantedState.speedMetersPerSecond / Constants.Modules.WHEEL_CIRCUMFERENCE;
    }

    /**
     * Compute the desired turn motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor position given wheel rotational position (rots)
     * @param wantedState the wanted state of the module
     * @return the desired rotor position
     * @see SwerveModuleState
     */
    public double computeDesiredTurnerRotations(final SwerveModuleState wantedState) {
        return wantedState.angle.getRotations();
    }

    /**
     * Set the desired {@link SwerveModuleState} of the module
     * @param state the desired {@link SwerveModuleState}
     * @see SwerveModuleState
     */
    public void setDesiredState(final SwerveModuleState state) {
        final Rotation2d currentWheelRotation = getAngle();

        final SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        if (Constants.Swerve.USE_SWERVE_COSINE_SCALING) {
            SwerveModule.scaleWithErrorCosine(wantedState, currentWheelRotation);
        }

        final double desiredDriverVelocity = computeDesiredDriverVelocity(wantedState);
        final double desiredTurnerRotations = computeDesiredTurnerRotations(wantedState);

        this.lastDesiredState = wantedState;
        moduleIO.setInputs(desiredDriverVelocity, desiredTurnerRotations);
    }

    /**
     * Get the last desired {@link SwerveModuleState} set in {@link SwerveModule#setDesiredState(SwerveModuleState)}
     * <p>
     * Note: this {@link SwerveModuleState} has been optimized and does not guarantee that it matches the last set state
     * @return the last desired {@link SwerveModuleState}
     */
    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }

    /**
     * @see SwerveModuleIO#setNeutralMode(NeutralModeValue)
     */
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        moduleIO.setNeutralMode(neutralMode);
    }

    /**
     * Abstraction to allow for less convoluted/repetitive construction of {@link SwerveModule}s with varying/different
     * hardware (i.e. motors, encoders, etc...)
     */
    public static class Builder {
        /**
         * Make a SDS MK4i {@link SwerveModule} with 2 Falcon500s ({@link TalonFX}s) for the drive and turn motors
         * and a {@link CANcoder} as the turn encoder.
         * @param name the name of the {@link SwerveModule}
         * @param driveMotor the drive {@link TalonFX}
         * @param turnMotor the turn {@link TalonFX}
         * @param canCoder the turn {@link CANcoder}
         * @param driveMotorInvertedValue {@link InvertedValue} of the drive {@link TalonFX}
         * @param turnMotorInvertedValue {@link InvertedValue} of the turn {@link TalonFX}
         * @param magnetOffset the magnet offset of the turn {@link CANcoder}
         * @param robotMode the {@link Constants.RobotMode} describing the current mode
         * @return the constructed {@link SwerveModule}
         */
        public static SwerveModule SDSMK4iFalcon500CANCoder(
                final String name,
                final TalonFX driveMotor,
                final TalonFX turnMotor,
                final CANcoder canCoder,
                final InvertedValue driveMotorInvertedValue,
                final InvertedValue turnMotorInvertedValue,
                final double magnetOffset,
                final Constants.RobotMode robotMode
        ) {
            final SwerveModuleIO swerveModuleIO = switch (robotMode) {
                case REAL -> new SwerveModuleIOFalcon(
                        driveMotor, turnMotor, canCoder,
                        driveMotorInvertedValue, turnMotorInvertedValue, magnetOffset
                );
                case SIM -> new SwerveModuleIOFalconSim(
                        driveMotor, turnMotor, canCoder,
                        driveMotorInvertedValue, turnMotorInvertedValue, magnetOffset
                );
                case REPLAY -> new SwerveModuleIO() {};
            };

            return new SwerveModule(swerveModuleIO, name);
        }

        /**
         * Make a SDS MK4i {@link SwerveModule} with 2 {@link TitanSparkMAX}s for the drive and turn motors and
         * an {@link com.revrobotics.AbsoluteEncoder} as the turn encoder.
         * @param name the name of the {@link SwerveModule}
         * @param driveMotor the drive {@link TitanSparkMAX}
         * @param turnMotor the turn {@link TitanSparkMAX}
         * @param driveMotorInvertedValue {@link InvertedValue} of the drive {@link TitanSparkMAX}
         * @param turnMotorInvertedValue {@link InvertedValue} of the turn {@link TitanSparkMAX}
         * @param magnetOffset the magnet offset of the turn {@link com.revrobotics.AbsoluteEncoder}
         * @param robotMode the {@link Constants.RobotMode} describing the current mode
         * @return the constructed {@link SwerveModule}
         */
        public static SwerveModule SDSMk4iSparkMAX(
                final String name,
                final TitanSparkMAX driveMotor,
                final TitanSparkMAX turnMotor,
                final InvertedValue driveMotorInvertedValue,
                final InvertedValue turnMotorInvertedValue,
                final double magnetOffset,
                final Constants.RobotMode robotMode
        ) {
            final SwerveModuleIO swerveModuleIO = switch (robotMode) {
                case REAL -> new SwerveModuleIONeo(
                        driveMotor, turnMotor,
                        driveMotorInvertedValue, turnMotorInvertedValue, magnetOffset
                );
                case SIM -> new SwerveModuleIONeoSim(
                        driveMotor, turnMotor,
                        driveMotorInvertedValue, turnMotorInvertedValue, magnetOffset
                );
                case REPLAY -> new SwerveModuleIO() {};
            };

            return new SwerveModule(swerveModuleIO, name);
        }
    }
}
