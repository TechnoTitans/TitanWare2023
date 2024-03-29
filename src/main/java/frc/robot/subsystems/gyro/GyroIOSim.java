package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.SwerveModule;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
    public static final double USE_SIMULATED_PITCH = 0;
    public static final double USE_SIMULATED_ROLL = 0;

    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSimState;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] swerveModules;
    private final double[] lastSwerveModulePositionMeters = {0.0, 0.0, 0.0, 0.0};

    private Pose2d gyroUseOdometryPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    // Cached StatusSignals
    private final StatusSignal<Boolean> _faultHardware;
    private final StatusSignal<Double> _yaw;
    private final StatusSignal<Double> _pitch;
    private final StatusSignal<Double> _roll;
    private final StatusSignal<Double> _yawVelocity;
    private final StatusSignal<Double> _pitchVelocity;
    private final StatusSignal<Double> _rollVelocity;

    public GyroIOSim(final Pigeon2 pigeon, final SwerveDriveKinematics kinematics, final SwerveModule[] swerveModules) {
        this.pigeon = pigeon;
        this.pigeonSimState = pigeon.getSimState();
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;

        this._faultHardware = pigeon.getFault_Hardware();
        this._yaw = pigeon.getYaw();
        this._pitch = pigeon.getPitch();
        this._roll = pigeon.getRoll();
        this._yawVelocity = pigeon.getAngularVelocityZ();
        this._pitchVelocity = pigeon.getAngularVelocityX();
        this._rollVelocity = pigeon.getAngularVelocityY();

        pigeonSimState.setSupplyVoltage(12);
        pigeonSimState.setPitch(USE_SIMULATED_PITCH);
        pigeonSimState.setRoll(USE_SIMULATED_ROLL);
    }

    private void updateGyro() {
        final SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            final SwerveModulePosition currentSwervePosition = swerveModules[i].getPosition();

            wheelDeltas[i] = new SwerveModulePosition(
                    (currentSwervePosition.distanceMeters - lastSwerveModulePositionMeters[i]),
                    currentSwervePosition.angle
            );
            lastSwerveModulePositionMeters[i] = currentSwervePosition.distanceMeters;
        }

        final Twist2d wheelDeltasTwist = kinematics.toTwist2d(wheelDeltas);
        gyroUseOdometryPose = gyroUseOdometryPose.exp(wheelDeltasTwist);

        Logger.getInstance().recordOutput(Gyro.logKey + "/GyroUseOdometryPose", gyroUseOdometryPose);
        Logger.getInstance().recordOutput(Gyro.logKey + "/WheelDeltasTwistDx", wheelDeltasTwist.dx);
        Logger.getInstance().recordOutput(Gyro.logKey + "/WheelDeltasTwistDy", wheelDeltasTwist.dy);
        Logger.getInstance().recordOutput(Gyro.logKey + "/WheelDeltasTwistDTheta", wheelDeltasTwist.dtheta);
        Logger.getInstance().recordOutput(Gyro.logKey + "/LastSwerveModulePositionMeters", lastSwerveModulePositionMeters);

        setAngleInternal(gyroUseOdometryPose.getRotation().getDegrees());
    }

    @Override
    public void config() {
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        //TODO: do we need to use MountPose?
        pigeon2Configuration.MountPose.MountPoseYaw = 0;

        pigeon.getConfigurator().apply(pigeon2Configuration);
    }

    @Override
    public void periodic() {
        updateGyro();
    }

    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        inputs.hasHardwareFault = _faultHardware.refresh().getValue();

        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = getPitch();
        inputs.rollPositionDeg = getRoll();

        inputs.yawVelocityDegPerSec = _yawVelocity.refresh().getValue();
        inputs.pitchVelocityDegPerSec = _pitchVelocity.refresh().getValue();
        inputs.rollVelocityDegPerSec = _rollVelocity.refresh().getValue();
    }

    public double getYaw() {
        // TODO: fairly sure that yaw, pitch, roll velocity don't work in sim, so we can't latency compensate
//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                _yaw,
//                getYawVelocitySignal()
//        );
        return _yaw.refresh().getValue();
    }

    public double getPitch() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getPitch(),
//                getPitchVelocitySignal()
//        );
        return _pitch.refresh().getValue();
    }

    public double getRoll() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getRoll(),
//                getRollVelocitySignal()
//        );
        return _roll.refresh().getValue();
    }

    private void setAngleInternal(final double angle) {
        pigeonSimState.setRawYaw(angle);
    }

    @Override
    public void setAngle(final Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}
