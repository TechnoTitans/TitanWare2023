package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double drivePositionRots = 0.0;
        public double driveVelocityRotsPerSec = 0.0;
        public double driveDesiredVelocityRotsPerSec = 0.0;

        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        public double turnAbsolutePositionRots = 0.0;
        public double turnDesiredAbsolutePositionRots = 0.0;
        public double turnVelocityRotsPerSec = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see SwerveModuleIOInputs
     * @see AutoLog
     */
    void updateInputs(final SwerveModuleIOInputs inputs);

    /**
     * Get if the current SwerveModuleIO is real
     * @return true if real, false if not
     */
    boolean isReal();

    /**
     * Periodic call, does <b>NOT</b> call updateInputs
     */
    default void periodic() {}

    /**
     * Config motors call, should only be invoked once on initialize
     */
    void config();

    /**
     * Get a {@link Rotation2d} of the current absolute turn position (computed from encoder rotations)
     * @return the absolute turn position as a {@link Rotation2d}
     * @see Rotation2d
     */
    Rotation2d getAngle();

    /**
     * Get the current relative drive wheel (mechanism) position in rotations
     * @return drive wheel position (rots)
     */
    double getDrivePosition();

    /**
     * Get the current drive wheel (mechanism) velocity in rotations/sec
     * @return drive wheel velocity (rps)
     */
    double getDriveVelocity();

    /**
     * Get the current module observed {@link SwerveModuleState} (velocity, angle)
     * Velocity is wheel linear velocity, angle is wheel absolute position
     * @return the module's current state as a {@link SwerveModuleState}
     * @see SwerveModuleState
     */
    SwerveModuleState getState();

    /**
     * Get the current module observed {@link SwerveModulePosition} (position, angle)
     * Velocity is wheel linear position, angle is wheel absolute position
     * @return the module's current position as a {@link SwerveModulePosition}
     * @see SwerveModulePosition
     */
    SwerveModulePosition getPosition();

    /**
     * Compute the desired drive motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor velocity given wheel velocity (rps)
     * @param wantedState the wanted state of the module
     * @return the desired rotor velocity
     * @see SwerveModuleState
     */
    double compute_desired_driver_velocity(final SwerveModuleState wantedState);

    /**
     * Compute the desired turn motor velocity given a desired {@link SwerveModuleState}
     * i.e. the rotor position given wheel rotational position (rots)
     * @param wantedState the wanted state of the module
     * @return the desired rotor position
     * @see SwerveModuleState
     */
    double compute_desired_turner_rotations(final SwerveModuleState wantedState);

    /**
     * Set the desired {@link SwerveModuleState} of the module
     * @param state the desired {@link SwerveModuleState}
     * @see SwerveModuleState
     */
    void setDesiredState(final SwerveModuleState state);

    /**
     * Stop the module
     */
    void stop();

    /**
     * Get the last desired {@link SwerveModuleState} set in {@link SwerveModuleIO#setDesiredState(SwerveModuleState)}
     * <p>
     * Note: this {@link SwerveModuleState} has been optimized and does not guarantee that it matches the last set state
     * @return the last desired {@link SwerveModuleState}
     */
    SwerveModuleState getLastDesiredState();

    void setNeutralMode(final NeutralModeValue neutralMode);
}
