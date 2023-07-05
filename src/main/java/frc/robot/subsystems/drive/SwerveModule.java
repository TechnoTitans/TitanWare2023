package frc.robot.subsystems.drive;

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
        moduleIO.updateInputs(inputs);
        Logger.getInstance().processInputs(String.format("Drive/Module_%s", name), inputs);
    }

    private void config() {
        moduleIO.config();
    }

    public Rotation2d getAngle() {
        return moduleIO.getAngle();
    }

    public double getDrivePosition() {
        return moduleIO.getDrivePosition();
    }

    public double getDriveVelocity() {
        return moduleIO.getDriveVelocity();
    }

    public SwerveModuleState getState() {
        return moduleIO.getState();
    }

    public SwerveModulePosition getPosition() {
        return moduleIO.getPosition();
    }

    public double compute_desired_driver_velocity(final SwerveModuleState wantedState) {
        return moduleIO.compute_desired_driver_velocity(wantedState);
    }

    public double compute_desired_turner_rotations(final SwerveModuleState wantedState) {
        return moduleIO.compute_desired_turner_rotations(wantedState);
    }

    public void setDesiredState(final SwerveModuleState state) {
        moduleIO.setDesiredState(state);
    }

    public void stop() {
        moduleIO.stop();
    }

    public SwerveModuleState getLastDesiredState() {
        return moduleIO.getLastDesiredState();
    }
}
