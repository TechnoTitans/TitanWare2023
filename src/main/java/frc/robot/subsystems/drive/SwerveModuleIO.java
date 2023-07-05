package frc.robot.subsystems.drive;

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
        public double turnDesiredAbsolutePositionRotsPerSec = 0.0;
        public double turnVelocityRotsPerSec = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    void updateInputs(final SwerveModuleIOInputs inputs);

    void config();

    Rotation2d getAngle();

    double getDrivePosition();

    double getDriveVelocity();

    SwerveModuleState getState();

    SwerveModulePosition getPosition();

    double compute_desired_driver_velocity(final SwerveModuleState wantedState);

    double compute_desired_turner_rotations(final SwerveModuleState wantedState);

    void setDesiredState(final SwerveModuleState state);

    void stop();

    SwerveModuleState getLastDesiredState();
}
