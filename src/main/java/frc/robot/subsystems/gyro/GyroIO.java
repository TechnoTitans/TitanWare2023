package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
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
    void updateInputs(final GyroIOInputs inputs);

    /**
     * Call to configure the Pigeon, should only be called once on init
     */
    void config();

    /**
     * Get the underlying Pigeon object from CTRE (no guarantees are made about real/sim)
     * @return the {@link Pigeon2}
     * @see Pigeon2
     */
    Pigeon2 getPigeon();

    /**
     * Get whether this Gyro is real (true/real if hardware exists, false if hardware does not exist - i.e. in a sim)
     * @return true if the Gyro is real, false if not
     */
    boolean isReal();

    /**
     * Get the current yaw (heading) reported by the Gyro
     * @return the current yaw (deg)
     */
    double getYaw();

    /**
     * Get the current yaw (heading) reported by the Gyro as a blocking call (waits for an update until timeout)
     * @return the current yaw (deg)
     */
    double getYawBlocking();

    /**
     * Get the current yaw (heading) as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current yaw
     * @see Rotation2d
     */
    default Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }


    /**
     * Get the current yaw (heading) as a {@link Rotation2d} by the Gyro
     * as a blocking call (waits for an update until timeout)
     * @return the {@link Rotation2d} of the current yaw
     * @see Rotation2d
     */
    default Rotation2d getYawRotation2dBlocking() {
        return Rotation2d.fromDegrees(getYawBlocking());
    }

    /**
     * Get the current pitch reported by the Gyro
     * @return the current pitch (deg)
     */
    double getPitch();

    /**
     * Get the current pitch as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current pitch
     * @see Rotation2d
     */
    default Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    /**
     * Get the current roll reported by the Gyro
     * @return the current roll (deg)
     */
    double getRoll();

    /**
     * Get the current pitch as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current pitch
     * @see Rotation2d
     */
    default Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    /**
     * Set the currently observed angle of the Gyro
     * @param angle the angle to set (deg)
     */
    void setAngle(final double angle);

    /**
     * Sets the current observed angle of the Gyro to 0 (deg)
     * @see GyroIO#setAngle(double)
     */
    default void zeroRotation() {
        setAngle(0);
    }
}
