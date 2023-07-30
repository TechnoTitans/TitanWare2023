package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean hasHardwareFault = false;
        public double rollPositionDeg = 0.0;
        public double pitchPositionDeg = 0.0;
        public double yawPositionDeg = 0.0;
        public double rollVelocityDegPerSec = 0.0;
        public double pitchVelocityDegPerSec = 0.0;
        public double yawVelocityDegPerSec = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see GyroIOInputs
     * @see AutoLog
     */
    default void updateInputs(final GyroIOInputs inputs) {}

    /**
     * Call to configure the Pigeon, should only be called once on init
     */
    default void config() {}

    /**
     * Set the currently observed angle of the Gyro
     * @param angle the angle to set (deg)
     */
    default void setAngle(final Rotation2d angle) {}

    /**
     * Sets the current observed angle of the Gyro to 0 (deg)
     * @see GyroIO#setAngle(Rotation2d)
     */
    default void zeroRotation() {
        setAngle(Rotation2d.fromDegrees(0));
    }
}
