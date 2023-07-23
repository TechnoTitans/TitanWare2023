package frc.robot.utils.gyro;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.function.Supplier;

public class GyroUtils {
    public static Rotation2d withAngleModulus(final Supplier<Rotation2d> rotation2dSupplier) {
        return Rotation2d.fromRadians(MathUtil.angleModulus(rotation2dSupplier.get().getRadians()));
    }

    public static double getAsDoubleDeg(final Rotation2d rotation2d) {
        return rotation2d.getDegrees();
    }

    public static double getAsDoubleDeg(final Supplier<Rotation2d> rotation2dSupplier) {
        return getAsDoubleDeg(rotation2dSupplier.get());
    }

    public static double getAsAngleModdedDoubleDeg(final Supplier<Rotation2d> rotation2dSupplier) {
        return getAsDoubleDeg(withAngleModulus(rotation2dSupplier));
    }

    public static Rotation3d rpyToRotation3d(final Rotation2d roll, final Rotation2d pitch, final Rotation2d yaw) {
        return new Rotation3d(roll.getRadians(), pitch.getRadians(), yaw.getRadians());
    }
}
