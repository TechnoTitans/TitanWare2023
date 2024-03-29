package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.sim.CurrentDrawSim;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Swerve extends SubsystemBase {
    protected static final String logKey = "Swerve";
    protected static final String odometryLogKey = "Odometry";
    private static final boolean _currentDrawEnabled = CurrentDrawSim.isEnabled();

    private Gyro gyro;
    private final GyroIOInputsAutoLogged gyroInputs;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveModule[] swerveModules;

    public Swerve(
            final Gyro gyro,
            final SwerveDriveKinematics kinematics,
            final SwerveModule frontLeft,
            final SwerveModule frontRight,
            final SwerveModule backLeft,
            final SwerveModule backRight
    ) {
        this.gyro = gyro;
        this.gyroInputs = new GyroIOInputsAutoLogged();

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.kinematics = kinematics;
        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                Constants.Vision.STATE_STD_DEVS,
                Constants.Vision.VISION_MEASUREMENT_STD_DEVS
        );

        this.swerveModules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};
    }

    /**
     * <p>
     * Takes in raw SwerveModuleStates and converts them such that the velocity components are converted to
     * their magnitudes (all positive) and the rotational components are all clamped to [0, 180]
     * </p>
     * <p>
     * This ensures that graphics/displays remain correct/easier to understand when under the case where
     * SwerveModuleStates are optimized and may be 180 degrees off (and the velocity component is negated)
     * </p>
     * <p>
     * Note: Do <b>NOT</b> use this for anything other than displaying SwerveModuleStates
     * </p>
     * @param swerveModuleStates raw SwerveModuleStates retrieved directly from the modules
     * @return modified SwerveModuleStates
     */
    private SwerveModuleState[] modifyModuleStatesForDisplay(final SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < swerveModuleStates.length; i++) {
            final SwerveModuleState origLastState = swerveModuleStates[i];
            final double origRots = origLastState.angle.getRotations();

            swerveModuleStates[i] = new SwerveModuleState(
                    Math.abs(origLastState.speedMetersPerSecond),
                    Rotation2d.fromRotations(MathUtil.inputModulus(origRots, 0, 1))
            );
        }

        return swerveModuleStates;
    }

    @Override
    public void periodic() {
        final double swervePeriodicUpdateStart = Logger.getInstance().getRealTimestamp();
        gyro.periodic();

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        Logger.getInstance().recordOutput(
                logKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getInstance().getRealTimestamp() - swervePeriodicUpdateStart)
        );

        //report current draws in sim
        if (_currentDrawEnabled) {
            CurrentDrawSim.getInstance().report(
                    Arrays.stream(swerveModules)
                            .map(SwerveModule::getCurrentDrawAmps)
                            .mapToDouble(Double::doubleValue)
                            .sum(),
                    Constants.PDH.DRIVETRAIN_CHANNELS
            );
        }

        //log current swerve chassis speeds
        final ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
        Logger.getInstance().recordOutput(
                logKey + "/LinearSpeedMetersPerSecond",
                Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        );
        Logger.getInstance().recordOutput(
                logKey + "/RobotRelativeChassisSpeeds", LogUtils.toDoubleArray(robotRelativeSpeeds)
        );
        Logger.getInstance().recordOutput(
                logKey + "/FieldRelativeChassisSpeeds", LogUtils.toDoubleArray(getFieldRelativeSpeeds())
        );

        //prep states for display
        final SwerveModuleState[] lastDesiredStates = modifyModuleStatesForDisplay(getModuleLastDesiredStates());
        final SwerveModuleState[] currentStates = modifyModuleStatesForDisplay(getModuleStates());

        Logger.getInstance().recordOutput(logKey + "/DesiredStates", lastDesiredStates);
        Logger.getInstance().recordOutput(logKey + "/CurrentStates", currentStates);

        // only update gyro from wheel odometry if we're not simulating and the gyro has failed
        if (Constants.CURRENT_MODE == Constants.RobotMode.REAL && gyroInputs.hasHardwareFault && gyro.isReal()) {
            final Pigeon2 pigeon2 = gyro.getPigeon();
            gyro = new Gyro(new GyroIOSim(pigeon2, kinematics, swerveModules), pigeon2);
        }

        Logger.getInstance().recordOutput(
               logKey + "/IsUsingFallbackSimGyro",
               Constants.CURRENT_MODE == Constants.RobotMode.REAL && !gyro.isReal()
        );

        // Update PoseEstimator and Odometry
        final double odometryUpdateStart = Logger.getInstance().getRealTimestamp();
        final Pose2d estimatedPosition = poseEstimator.update(getYaw(), getModulePositions());
        final double odometryUpdatePeriodMs = LogUtils.microsecondsToMilliseconds(
                Logger.getInstance().getRealTimestamp() - odometryUpdateStart
        );

        Logger.getInstance().recordOutput(
                odometryLogKey + "/OdometryUpdatePeriodMs", odometryUpdatePeriodMs
        );
        Logger.getInstance().recordOutput(odometryLogKey + "/Robot2d", estimatedPosition);
        Logger.getInstance().recordOutput(odometryLogKey + "/Robot3d", GyroUtils.robotPose2dToPose3dWithGyro(
                estimatedPosition,
                GyroUtils.rpyToRotation3d(getRoll(), getPitch(), getYaw())
        ));
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Get the estimated {@link Pose2d} of the robot from the {@link SwerveDrivePoseEstimator}.
     * @return the estimated position of the robot, as a {@link Pose2d}
     */
    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    public Gyro getGyro() {
        return gyro;
    }

    public Rotation2d getPitch() {
        return gyro.getPitchRotation2d();
    }

    public Rotation2d getRoll() {
        return gyro.getRollRotation2d();
    }

    public Rotation2d getYaw() {
        return gyro.getYawRotation2d();
    }

    /**
     * @see Gyro#setAngle(Rotation2d)
     */
    public void setAngle(final Rotation2d angle) {
        gyro.setAngle(angle);
    }

    public void zeroRotation(final PhotonVision<?> photonVision) {
        gyro.zeroRotation();
        photonVision.resetPosition(
                photonVision.getEstimatedPosition(),
                Rotation2d.fromDegrees(0)
        );
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                    frontLeft.getState(),
                    frontRight.getState(),
                    backLeft.getState(),
                    backRight.getState()
        );
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                getRobotRelativeSpeeds(),
                getYaw().times(-1)
        );
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public SwerveModuleState[] getModuleLastDesiredStates() {
        return new SwerveModuleState[] {
                frontLeft.getLastDesiredState(),
                frontRight.getLastDesiredState(),
                backLeft.getLastDesiredState(),
                backRight.getLastDesiredState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public void drive(final SwerveModuleState[] states, final double moduleMaxSpeed) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, moduleMaxSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void drive(final SwerveModuleState[] states) {
        drive(states, Constants.Swerve.MODULE_MAX_SPEED);
    }

    public void drive(
            final double xSpeed,
            final double ySpeed,
            final double rot,
            final boolean fieldRelative
    ) {
        final ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        drive(speeds);
    }

    public void drive(final ChassisSpeeds speeds, final double moduleMaxSpeed) {
        final ChassisSpeeds correctedSpeeds;
        if (Constants.Swerve.USE_SWERVE_SKEW_FIX) {
            // TODO: replace with ChassisSpeeds.discretize() once we get wpilib updated
            final double dtSeconds = Constants.LOOP_PERIOD_SECONDS;
            final Pose2d desiredDeltaPose = new Pose2d(
                    speeds.vxMetersPerSecond * dtSeconds,
                    speeds.vyMetersPerSecond * dtSeconds,
                    Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dtSeconds)
            );

            final Twist2d twist2d = new Pose2d().log(desiredDeltaPose);
            correctedSpeeds = new ChassisSpeeds(
                    twist2d.dx / dtSeconds,
                    twist2d.dy / dtSeconds,
                    twist2d.dtheta / dtSeconds
            );
        } else {
            correctedSpeeds = speeds;
        }

        drive(kinematics.toSwerveModuleStates(correctedSpeeds), moduleMaxSpeed);
    }

    public void drive(final ChassisSpeeds speeds) {
        drive(speeds, Constants.Swerve.MODULE_MAX_SPEED);
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    /**
     * Drive with dx and dy while facing a specified (non)holonomic rotation theta with a custom rotation kP
     * @param dx the desired dx component
     * @param dy the desired dy component
     * @param theta the desired holonomic (if fieldRelative) or non-holonomic (if not fieldRelative) rotation (deg)
     * @param fieldRelative true if driving should be field relative, false if not
     * @param rotation_kP custom kP component of the rotation
     */
    public void faceDirection(
            final double dx,
            final double dy,
            final Rotation2d theta,
            final boolean fieldRelative,
            final double rotation_kP
    ) {
        final Rotation2d error = theta.minus(getYaw());
        final Rotation2d rotPower = error.times(rotation_kP);

        drive(dx, dy, rotPower.getRadians(), fieldRelative);
    }

    /**
     * Drive with dx and dy while facing a specified (non)holonomic rotation theta
     * @param dx the desired dx component
     * @param dy the desired dy component
     * @param theta the desired holonomic (if fieldRelative) or non-holonomic (if not fieldRelative) rotation (deg)
     * @param fieldRelative true if driving should be field relative, false if not
     */
    public void faceDirection(
            final double dx,
            final double dy,
            final Rotation2d theta,
            final boolean fieldRelative
    ) {
        faceDirection(dx, dy, theta, fieldRelative, Constants.Swerve.ROTATE_P);
    }


    /**
     * Drive all modules to a raw {@link SwerveModuleState}
     * @param s1 speed of module 1 (m/s)
     * @param s2 speed of module 2 (m/s)
     * @param s3 speed of module 3 (m/s)
     * @param s4 speed of module 4 (m/s)
     * @param a1 angle of module 1 (deg)
     * @param a2 angle of module 2 (deg)
     * @param a3 angle of module 3 (deg)
     * @param a4 angle of module 4 (deg)
     * @see Swerve#drive(SwerveModuleState[])
     * @see SwerveModuleState
     */
    public void rawSet(
            final double s1,
            final double s2,
            final double s3,
            final double s4,
            final double a1,
            final double a2,
            final double a3,
            final double a4
    ) {
        drive(new SwerveModuleState[] {
                new SwerveModuleState(s1, Rotation2d.fromDegrees(a1)),
                new SwerveModuleState(s2, Rotation2d.fromDegrees(a2)),
                new SwerveModuleState(s3, Rotation2d.fromDegrees(a3)),
                new SwerveModuleState(s4, Rotation2d.fromDegrees(a4))
        });
    }

    /**
     * Zero all modules
     * @see Swerve#rawSet(double, double, double, double, double, double, double, double)
     */
    @SuppressWarnings("unused")
    public void zero() {
        rawSet(0, 0, 0, 0, 0, 0, 0, 0);
    }

    /**
     * Put modules into an X pattern (significantly reduces the swerve's ability to coast/roll)
     * @see Swerve#rawSet(double, double, double, double, double, double, double, double)
     */
    public void wheelX() {
        rawSet(0, 0, 0, 0, 45, -45, -45, 45);
    }

    /**
     * Set the desired {@link NeutralModeValue} of all module drive motors
     * @param neutralMode the desired {@link NeutralModeValue}
     * @see SwerveModule#setNeutralMode(NeutralModeValue)
     */
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        frontLeft.setNeutralMode(neutralMode);
        frontRight.setNeutralMode(neutralMode);
        backLeft.setNeutralMode(neutralMode);
        backRight.setNeutralMode(neutralMode);
    }
}
