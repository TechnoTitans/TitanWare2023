package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Consumer;

@SuppressWarnings("unused")
public class Swerve extends SubsystemBase {
    private final Pigeon2 pigeon;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    public Swerve(
            final Pigeon2 pigeon,
            final SwerveDriveKinematics kinematics,
            final SwerveModule frontLeft,
            final SwerveModule frontRight,
            final SwerveModule backLeft,
            final SwerveModule backRight
    ) {
        this.pigeon = pigeon;
        this.kinematics = kinematics;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public double getHeading() {
        return pigeon.getYaw();
    }

    public double getPitch() {
        return pigeon.getPitch();
    }

    public double getRoll() {
        return pigeon.getRoll();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void setAngle(final double angle) {
        pigeon.setYaw(angle);
    }

    public void zeroRotation() {
        setAngle(0);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
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

    public void drive(
            final double xSpeed,
            final double ySpeed,
            final double rot,
            final boolean fieldRelative
    ) {
        final ChassisSpeeds speeds = (fieldRelative)
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);
        drive(speeds);
    }

    public void drive(final ChassisSpeeds speeds) {
        drive(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(final SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MODULE_MAX_SPEED);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
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

    public void manualPercentOutput(final double percentOutput) {
        frontLeft.percentOutputControl(percentOutput);
        frontRight.percentOutputControl(percentOutput);
        backLeft.percentOutputControl(percentOutput);
        backRight.percentOutputControl(percentOutput);
    }

    public void manualVelocity(final double velocityTicksPer100ms) {
        frontLeft.manualVelocityControl(velocityTicksPer100ms);
        frontRight.manualVelocityControl(velocityTicksPer100ms);
        backLeft.manualVelocityControl(velocityTicksPer100ms);
        backRight.manualVelocityControl(velocityTicksPer100ms);
    }
}
