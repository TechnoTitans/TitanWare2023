package frc.robot.utils.logging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class LogUtils {
    public static double[] toDoubleArray(final SwerveModuleState swerveModuleState) {
        return new double[] {swerveModuleState.speedMetersPerSecond, swerveModuleState.angle.getRadians()};
    }

    public static SwerveModuleState fromDoubleArray(final double[] stateArray) {
        return new SwerveModuleState(stateArray[0], Rotation2d.fromRadians(stateArray[1]));
    }

    public static double[] toDoubleArray(final ChassisSpeeds chassisSpeeds) {
        return new double[] {
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond
        };
    }
}
