package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean hasHardwareFault = false;
        public double rollPositionRad = 0.0;
        public double pitchPositionRad = 0.0;
        public double yawPositionRad = 0.0;
        public double rollVelocityRadPerSec = 0.0;
        public double pitchVelocityRadPerSec = 0.0;
        public double yawVelocityRadPerSec = 0.0;
    }

    void updateInputs(final GyroIOInputs inputs);

    Pigeon2 getPigeon();

    double getHeading();

    double getPitch();

    double getRoll();

    default Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    void setAngle(final double angle);

    default void zeroRotation() {
        setAngle(0);
    }
}
