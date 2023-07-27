package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double[] lastDesiredStates = new double[] {};

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
    default void updateInputs(final SwerveModuleIOInputs inputs) {}

    /**
     * Periodic call, does <b>NOT</b> call updateInputs
     */
    default void periodic() {}

    /**
     * Config motors call, should only be invoked once on initialize
     */
    default void config() {}

    /**
     * Set the desired {@link SwerveModuleState} of the module
     * @param state the desired {@link SwerveModuleState}
     * @see SwerveModuleState
     */
    default void setDesiredState(final SwerveModuleState state) {}

    /**
     * Stop the module
     */
    default void stop() {}

    /**
     * Set the desired {@link NeutralModeValue} of the drive motor on this module
     * @param neutralMode the desired {@link NeutralModeValue}
     * @see NeutralModeValue
     */
    default void setNeutralMode(final NeutralModeValue neutralMode) {}
}
