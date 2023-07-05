package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

@SuppressWarnings("unused")
public class Swerve extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    private final double[] lastSwerveModulePositionMeters = {0.0, 0.0, 0.0, 0.0};
    private Pose2d fallbackOdometryPose = new Pose2d();

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
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        // only update gyro from wheel odometry if we're simulating or the gyro has failed
        if (Constants.CURRENT_MODE == Constants.RobotMode.SIM || gyroInputs.hasHardwareFault) {
            final SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};
            final SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
                wheelDeltas[i] = new SwerveModulePosition(
                        (swerveModules[i].getDrivePosition() - lastSwerveModulePositionMeters[i]),
                        swerveModules[i].getAngle()
                );
                lastSwerveModulePositionMeters[i] = swerveModules[i].getDrivePosition();
            }

            final Twist2d wheelDeltasTwist = kinematics.toTwist2d(wheelDeltas);
            fallbackOdometryPose = fallbackOdometryPose.exp(wheelDeltasTwist);
            gyroIO.setAngle(fallbackOdometryPose.getRotation().getDegrees());
        }
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

    // TODO: this needs to be somewhere else probably it was only temporary
    public void logModuleDesiredState(final SwerveModule module, final SwerveModuleState state) {
        final String rootName = "SwerveStates/DesiredStates/" + module.getName();
        Logger.getInstance().recordOutput(
                rootName + "/DriveVelocity", Math.abs(module.compute_desired_driver_velocity(state))
        );
        Logger.getInstance().recordOutput(
                rootName + "/TurnAbsolutePositionRots", module.compute_desired_turner_rotations(state)
        );
    }

    public void drive(final SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MODULE_MAX_SPEED);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);

//        logModuleDesiredState(frontLeft, frontLeft.getLastDesiredState());
//        logModuleDesiredState(frontRight, frontRight.getLastDesiredState());
//        logModuleDesiredState(backLeft, backLeft.getLastDesiredState());
//        logModuleDesiredState(backRight, backRight.getLastDesiredState());
    }

    public void drive(
            final double xSpeed,
            final double ySpeed,
            final double rot,
            final boolean fieldRelative
    ) {
        ChassisSpeeds speeds = (fieldRelative)
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        drive(speeds);
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
        final Rotation2d error = Rotation2d.fromDegrees(theta).minus(Rotation2d.fromDegrees(-getHeading()));
        final double rotPower = error.getRadians() * Constants.Swerve.ROTATE_P;
        drive(dx, dy, rotPower, fieldRelative);
    }

    public void faceDirection(
            final double dx,
            final double dy,
            final double theta,
            final boolean fieldRelative,
            final double kP
    ) {
        final Rotation2d error = Rotation2d.fromDegrees(theta).minus(Rotation2d.fromDegrees(-getHeading()));
        final double rotPower = error.getRadians() * kP;
        drive(dx, dy, rotPower, fieldRelative);
    }

    public void zeroWheels() {
        drive(new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        });
    }
}
