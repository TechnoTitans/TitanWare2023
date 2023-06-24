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

    public Swerve(Pigeon2 pigeon, SwerveDriveKinematics kinematics, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) {
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

    public void setAngle(double angle) {
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

    public SwerveModuleState[] getModuleDesiredStates() {
        return new SwerveModuleState[] {
                frontLeft.getLastDesiredState(),
                frontRight.getLastDesiredState(),
                backLeft.getLastDesiredState(),
                backRight.getLastDesiredState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    }

    public Consumer<SwerveModuleState[]> getModuleStateConsumer() {
        return this::drive;
    }

    public void tankDrive(double lspeed, double rspeed) {
        frontLeft.percentOutputControl(lspeed);
        backLeft.percentOutputControl(lspeed);
        frontRight.percentOutputControl(-rspeed);
        backRight.percentOutputControl(-rspeed);
    }

    public void drive(double xspeed, double yspeed, double rot, boolean fieldRelative) {
//        ChassisSpeeds speeds = (fieldRelative) ?
//                ChassisSpeeds.fromFieldRelativeSpeeds(xspeed, yspeed, rot, getRotation2d()) :
//                new ChassisSpeeds(xspeed, yspeed, rot);
//
//        // This is supposed to correct for the actuation speed of the swerve. Should try and prevent robot drift.
//        // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
//        double dtConstant = 0.009; //This is a real magic number.
//        Pose2d robotPoseVel = new Pose2d(
//                speeds.vxMetersPerSecond * dtConstant,
//                speeds.vyMetersPerSecond * dtConstant,
//                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dtConstant));
//        Twist2d twistVel = PoseUtils.PoseLog(robotPoseVel);
//
//        speeds = new ChassisSpeeds(twistVel.dx / dtConstant, twistVel.dy / dtConstant,
//                twistVel.dtheta / dtConstant);
//
//        drive(speeds);

        ChassisSpeeds speeds = (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(xspeed, yspeed, rot, getRotation2d()) : new ChassisSpeeds(xspeed, yspeed, rot);
        drive(speeds);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MODULE_MAX_SPEED);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void brake() {
        frontLeft.brake();
        backLeft.brake();
        frontRight.brake();
        backRight.brake();
    }

    public void coast() {
        frontLeft.coast();
        backLeft.coast();
        frontRight.coast();
        backRight.coast();
    }

    public void stop() {
        frontLeft.stop();
        backLeft.stop();
        frontRight.stop();
        backRight.stop();
    }

    public void faceDirection(double dx, double dy, double theta, boolean fieldRelative) {
        Rotation2d error = Rotation2d.fromDegrees(theta).minus(Rotation2d.fromDegrees(-getHeading()));
        double rotPower = error.getRadians() * Constants.Swerve.ROTATE_P;
        drive(dx, dy, rotPower, fieldRelative);
    }

    public void faceDirection(double dx, double dy, double theta, boolean fieldRelative, double kP) {
        Rotation2d error = Rotation2d.fromDegrees(theta).minus(Rotation2d.fromDegrees(-getHeading()));
        double rotPower = error.getRadians() * kP;
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

    public void manualPercentOutput(double percentOutput) {
        frontLeft.percentOutputControl(percentOutput);
        frontRight.percentOutputControl(percentOutput);
        backLeft.percentOutputControl(percentOutput);
        backRight.percentOutputControl(percentOutput);
    }

    public void manualVelocity(double velocityTicksPer100ms) {
        frontLeft.manualVelocityControl(velocityTicksPer100ms);
        frontRight.manualVelocityControl(velocityTicksPer100ms);
        backLeft.manualVelocityControl(velocityTicksPer100ms);
        backRight.manualVelocityControl(velocityTicksPer100ms);
    }
}
