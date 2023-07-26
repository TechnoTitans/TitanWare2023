package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String name;
    private final SwerveModuleIO moduleIO;
    private final SwerveModuleIOInputsAutoLogged inputs;


    public SwerveModule(final SwerveModuleIO moduleIO, final String name) {
        this.name = name;
        this.moduleIO = moduleIO;
        this.inputs = new SwerveModuleIOInputsAutoLogged();

        config();
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
     * @see SwerveModuleIO#getAngle()
     */
    public Rotation2d getAngle() {
        return moduleIO.getAngle();
    }

    /**
     * @see SwerveModuleIO#getDrivePosition()
     */
    public double getDrivePosition() {
        return moduleIO.getDrivePosition();
    }

    /**
     * @see SwerveModuleIO#getDriveVelocity()
     */
    public double getDriveVelocity() {
        return moduleIO.getDriveVelocity();
    }

    /**
     * @see SwerveModuleIO#getState()
     */
    public SwerveModuleState getState() {
        return moduleIO.getState();
    }

    /**
     * @see SwerveModuleIO#getPosition()
     */
    public SwerveModulePosition getPosition() {
        return moduleIO.getPosition();
    }

    /**
     * @see SwerveModuleIO#compute_desired_driver_velocity(SwerveModuleState)
     */
    public double compute_desired_driver_velocity(final SwerveModuleState wantedState) {
        return moduleIO.compute_desired_driver_velocity(wantedState);
    }

    /**
     * @see SwerveModuleIO#compute_desired_turner_rotations(SwerveModuleState)
     */
    public double compute_desired_turner_rotations(final SwerveModuleState wantedState) {
        return moduleIO.compute_desired_turner_rotations(wantedState);
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
     * @see SwerveModuleIO#getLastDesiredState()
     */
    public SwerveModuleState getLastDesiredState() {
        return moduleIO.getLastDesiredState();
    }

    /**
     * @see SwerveModuleIO#setNeutralMode(NeutralModeValue)
     */
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        moduleIO.setNeutralMode(neutralMode);
    }
}
