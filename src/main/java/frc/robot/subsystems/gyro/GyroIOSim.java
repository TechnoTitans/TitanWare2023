package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.SwerveModuleIO;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
    private final Pigeon2 pigeon;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModuleIO[] swerveModules;
    private final double[] lastSwerveModulePositionMeters = {0.0, 0.0, 0.0, 0.0};
    private Pose2d gyroUseOdometryPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static final double USE_SIMULATED_PITCH = 0;
    public static final double USE_SIMULATED_ROLL = 0;


    public GyroIOSim(final Pigeon2 pigeon, final SwerveDriveKinematics kinematics, final SwerveModuleIO[] swerveModules) {
        this.pigeon = pigeon;
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;

        pigeon.getSimState().setSupplyVoltage(12);
        pigeon.getSimState().setPitch(USE_SIMULATED_PITCH);
        pigeon.getSimState().setRoll(USE_SIMULATED_ROLL);
    }

    //TODO: figure out why wheelDeltasTwist is a zero twist but still works
    private void updateGyro() {
        final SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (swerveModules[i].getDrivePosition() - lastSwerveModulePositionMeters[i]),
                    swerveModules[i].getAngle()
            );
            lastSwerveModulePositionMeters[i] = swerveModules[i].getDrivePosition();
        }

        final Twist2d wheelDeltasTwist = kinematics.toTwist2d(wheelDeltas);

//        final Twist2d zeroTwist = new Twist2d(0, 0, 0);
//        final boolean isEqualToZero = wheelDeltasTwist.equals(zeroTwist);

        Logger.getInstance().recordOutput("gyroUseOdometryPose", gyroUseOdometryPose);
        Logger.getInstance().recordOutput("wheelDeltasTwistDx", wheelDeltasTwist.dx);
        Logger.getInstance().recordOutput("wheelDeltasTwistDy", wheelDeltasTwist.dy);
        Logger.getInstance().recordOutput("wheelDeltasTwistDTheta", wheelDeltasTwist.dtheta);
//        Logger.getInstance().recordOutput("wheelDeltasTwistIsZero", isEqualToZero);

//        Logger.getInstance().recordOutput("isDxZero", wheelDeltasTwist.dx == 0);
//        Logger.getInstance().recordOutput("isDyRZero", wheelDeltasTwist.dy == 0);
//        Logger.getInstance().recordOutput("isDThetaZero", wheelDeltasTwist.dtheta == 0);

        Logger.getInstance().recordOutput("lastSwerveModulePositionMeters", lastSwerveModulePositionMeters);

        gyroUseOdometryPose = gyroUseOdometryPose.exp(wheelDeltasTwist);
        setAngle(gyroUseOdometryPose.getRotation().getDegrees());
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        updateGyro();

        inputs.hasHardwareFault = pigeon.getFault_Hardware().refresh().getValue();

        inputs.rollPositionDeg = getRoll();
        inputs.pitchPositionDeg = -getPitch();
        inputs.yawPositionDeg = getHeading();

        inputs.rollVelocityDegPerSec = pigeon.getAngularVelocityY().refresh().getValue();
        inputs.pitchVelocityDegPerSec = -pigeon.getAngularVelocityX().refresh().getValue();
        inputs.yawVelocityDegPerSec = pigeon.getAngularVelocityZ().refresh().getValue();
    }

    @Override
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    @Override
    public boolean isReal() { return false; }

    @Override
    public double getHeading() {
        updateGyro();
        return pigeon.getYaw().refresh().getValue();
    }

    @Override
    public double getPitch() {
        // pigeon is in simulation, pitch is not going to work, so just assume 0
        return USE_SIMULATED_PITCH;
    }

    @Override
    public double getRoll() {
        // pigeon is in simulation, roll is not going to work, so just assume 0
        return USE_SIMULATED_ROLL;
    }

    @Override
    public void setAngle(final double angle) {
        pigeon.getSimState().setRawYaw(angle);
    }
}