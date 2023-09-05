package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
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
        this.inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public String getName() { return name; }

    public void periodic() {
        moduleIO.periodic();

        moduleIO.updateInputs(inputs);
        Logger.getInstance().processInputs(logKey, inputs);

        Logger.getInstance().recordOutput(logKey + "/CurrentState", getState());
        Logger.getInstance().recordOutput(logKey + "/LastDesiredState", lastDesiredState);
        Logger.getInstance().recordOutput(
                logKey + "/DriveDesiredVelocityRotsPerSec",
                computeDesiredDriverVelocity(lastDesiredState)
        );

        Logger.getInstance().recordOutput(
                logKey + "/TurnDesiredAbsolutePositionRots",
                computeDesiredTurnerRotations(lastDesiredState)
        );
    }

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
}
