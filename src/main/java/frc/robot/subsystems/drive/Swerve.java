package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.utils.PoseUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

@SuppressWarnings("unused")
public class Swerve extends SubsystemBase {
    private GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    private final SwerveModuleIO[] swerveModuleIOs;

    public Swerve(
            final GyroIO gyroIO,
            final SwerveDriveKinematics kinematics,
            final SwerveModuleIO frontLeftIO,
            final SwerveModuleIO frontRightIO,
            final SwerveModuleIO backLeftIO,
            final SwerveModuleIO backRightIO
    ) {
        this.gyroIO = gyroIO;
        this.gyroInputs = new GyroIOInputsAutoLogged();

        this.kinematics = kinematics;
        this.frontLeft = new SwerveModule(frontLeftIO, "FrontLeft");
        this.frontRight = new SwerveModule(frontRightIO, "FrontRight");
        this.backLeft = new SwerveModule(backLeftIO, "BackLeft");
        this.backRight = new SwerveModule(backRightIO, "BackRight");

        this.swerveModuleIOs = new SwerveModuleIO[] {frontLeftIO, frontRightIO, backLeftIO, backRightIO};
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
                    Rotation2d.fromRotations(origRots + ((origRots < 0) ? 1 : 0))
            );
        }

        return swerveModuleStates;
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        //prep states for display
        final SwerveModuleState[] lastDesiredStates = modifyModuleStatesForDisplay(getModuleLastDesiredStates());
        final SwerveModuleState[] currentStates = modifyModuleStatesForDisplay(getModuleStates());

        Logger.getInstance().recordOutput("SwerveStates/DesiredStates", lastDesiredStates);
        Logger.getInstance().recordOutput("SwerveStates/CurrentStates", currentStates);

        // only update gyro from wheel odometry if we're not simulating and the gyro has failed
        if (Constants.CURRENT_MODE == Constants.RobotMode.REAL && gyroInputs.hasHardwareFault && gyroIO.isReal()) {
            gyroIO = new GyroIOSim(gyroIO.getPigeon(), kinematics, swerveModuleIOs);
        }

        Logger.getInstance().recordOutput(
                "State/IsUsingFallbackSimGyro",
               Constants.CURRENT_MODE == Constants.RobotMode.REAL && !gyroIO.isReal()
        );
    }

    public double getHeading() {
        return gyroIO.getHeading();
    }

    public double getPitch() {
        return gyroIO.getPitch();
    }

    public double getRoll() {
        return gyroIO.getRoll();
    }

    public Rotation2d getRotation2d() {
        return gyroIO.getRotation2d();
    }

    /**
     * @see GyroIO#setAngle(double)
     */
    public void setAngle(final double angle) {
        gyroIO.setAngle(angle);
    }

    public void zeroRotation() {
        gyroIO.zeroRotation();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
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

    public Consumer<SwerveModuleState[]> getModuleStateConsumer() {
        return this::drive;
    }

    public Consumer<ChassisSpeeds> getChassisSpeedsConsumer() {
        return this::drive;
    }

    public void drive(final SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MODULE_MAX_SPEED);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void drive(
            final double xSpeed,
            final double ySpeed,
            final double rot,
            final boolean fieldRelative
    ) {
        final ChassisSpeeds speeds = (fieldRelative)
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        final ChassisSpeeds correctedSpeeds;
        if (Constants.Swerve.USE_SWERVE_SKEW_FIX) {
            final Pose2d lookaheadRobotPose = new Pose2d(
                    speeds.vxMetersPerSecond * Constants.LOOP_PERIOD_SECONDS,
                    speeds.vyMetersPerSecond * Constants.LOOP_PERIOD_SECONDS,
                    Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.LOOP_PERIOD_SECONDS)
            );

            final Twist2d twist2d = PoseUtils.log(lookaheadRobotPose);
            correctedSpeeds = new ChassisSpeeds(
                    twist2d.dx / Constants.LOOP_PERIOD_SECONDS,
                    twist2d.dy / Constants.LOOP_PERIOD_SECONDS,
                    twist2d.dtheta / Constants.LOOP_PERIOD_SECONDS
            );
        } else {
            correctedSpeeds = speeds;
        }

        drive(correctedSpeeds);
    }

    public void drive(final ChassisSpeeds speeds) {
        drive(kinematics.toSwerveModuleStates(speeds));
    }

    public void stop() {
        frontLeft.stop();
        backLeft.stop();
        frontRight.stop();
        backRight.stop();
    }

    public void faceDirection(
            final double dx,
            final double dy,
            final double theta,
            final boolean fieldRelative
    ) {
        final Rotation2d error = Rotation2d.fromDegrees(theta).minus(getRotation2d());
        final double rotPower = error.getRadians() * Constants.Swerve.ROTATE_P;
        drive(dx, dy, rotPower, fieldRelative);
    }

    public void faceDirection(
            final double dx,
            final double dy,
            final double theta,
            final boolean fieldRelative,
            final double rotation_kP
    ) {
        final Rotation2d error = Rotation2d.fromDegrees(theta).minus(getRotation2d());
        final double rotPower = error.getRadians() * rotation_kP;
        drive(dx, dy, rotPower, fieldRelative);
    }

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

    public void zero() {
        rawSet(0, 0, 0, 0, 0, 0, 0, 0);
    }

    public void wheelX() {
        rawSet(0, 0, 0, 0, 45, -45, 45, -45);
    }
}
