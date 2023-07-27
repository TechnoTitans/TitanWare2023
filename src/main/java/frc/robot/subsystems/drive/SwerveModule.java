package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String name;
    private final SwerveModuleIO moduleIO;
    private final SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModule(final SwerveModuleIO moduleIO, final String name) {
        this.name = name;
        this.moduleIO = moduleIO;
        this.inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public String getName() { return name; }

    public void periodic() {
        moduleIO.periodic();
        moduleIO.updateInputs(inputs);
        Logger.getInstance().processInputs(String.format("Drive/Module_%s", name), inputs);
    }

    /**
     * @see SwerveModuleIO#config()
     */
    private void config() {
        moduleIO.config();
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
     * @see SwerveModuleIO#setDesiredState(SwerveModuleState)
     */
    public void setDesiredState(final SwerveModuleState state) {
        moduleIO.setDesiredState(state);
    }

    /**
     * @see SwerveModuleIO#stop()
     */
    public void stop() {
        moduleIO.stop();
    }

    /**
     * Get the last desired {@link SwerveModuleState} set in {@link SwerveModuleIO#setDesiredState(SwerveModuleState)}
     * <p>
     * Note: this {@link SwerveModuleState} has been optimized and does not guarantee that it matches the last set state
     * @return the last desired {@link SwerveModuleState}
     */
    public SwerveModuleState getLastDesiredState() {
        return LogUtils.fromDoubleArray(inputs.lastDesiredStates);
    }

    /**
     * @see SwerveModuleIO#setNeutralMode(NeutralModeValue)
     */
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        moduleIO.setNeutralMode(neutralMode);
    }
}
