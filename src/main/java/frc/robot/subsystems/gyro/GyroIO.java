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
     * Get the current heading (yaw) reported by the Gyro
     * @return the current heading (deg)
     */
    double getHeading();

    //TODO: I DONT WANNA DO SO HARRY WILL MAKE AP~LANG DOCUMENTATION HERE!!!
    double getHeadingBlocking();

    /**
     * Get the current pitch reported by the Gyro
     * @return the current pitch (deg)
     */
    double getPitch();

    /**
     * Get the current roll reported by the Gyro
     * @return the current roll (deg)
     */
    double getRoll();

    /**
     * Get the current heading (yaw) as a {@link Rotation2d}
     * @return the {@link Rotation2d} of the current heading
     * @see Rotation2d
     */
    Rotation2d getRotation2d();

    //TODO: HARRINTGTON PLS DOCUMMENT
    Rotation2d getRotation2dBlocking();

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
