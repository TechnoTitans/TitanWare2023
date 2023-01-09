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
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveDriveKinematics kinematics;

    public Swerve(Pigeon2 pigeon, SwerveDriveKinematics kinematics, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) {
        this.pigeon = pigeon;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = kinematics;
    }

    public double getAngle() {
        return pigeon.getYaw();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getAngle());
    }

    public void setAngle(double angle) {
        pigeon.setYaw(angle);
    }

    public void zeroRotation() {
        pigeon.setYaw(0);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    }

    public Consumer<SwerveModuleState[]> getModuleStatesConsumer() {
        return swerveModuleStates -> {
            frontLeft.getState();
            frontRight.getState();
            backLeft.getState();
            backRight.getState();
        };
    }

    public void tankDrive(double lspeed, double rspeed) {
        frontLeft.percentOutputControl(lspeed);
        frontRight.percentOutputControl(-rspeed);
        backLeft.percentOutputControl(lspeed);
        backRight.percentOutputControl(-rspeed);
    }

    public void drive(double xspeed, double yspeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds = (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(xspeed, yspeed, rot, getHeading()) : new ChassisSpeeds(xspeed, yspeed, rot);
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

    public void stop() {
        SwerveModuleState stopped = new SwerveModuleState(0, null);
    }

    //Needs to be called in execute
    public void faceDirection(double dx, double dy, double theta, boolean fieldRelative) {
        double errorTheta = (theta - (int) getAngle() % 360);

        if (errorTheta < -180) errorTheta += 360;
        if (errorTheta > 180) errorTheta -= 360;
        if (Math.abs(errorTheta) < 5) errorTheta = 0;

        double pRotation = errorTheta * Constants.Swerve.ROTATE_P;

        if (Math.abs(pRotation) > Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED)
            pRotation = Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED * ((pRotation > 0) ? 1 : -1);

        drive(dx, dy, -pRotation, fieldRelative);
    }

    //Needs to be called in execute
    public void faceClosest(double dx, double dy, boolean fieldRelative) {
        int current_rotation = (int) getAngle() % 360;
        if (current_rotation < 0) current_rotation += 360;

        if (current_rotation <= 90 || current_rotation >= 270) {
            faceDirection(dx, dy, 0, fieldRelative);
        } else {
            faceDirection(dx, dy, 180, fieldRelative);
        }
    }

    public void tuneTurner(int desiredAngle) {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(desiredAngle)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(desiredAngle)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(desiredAngle)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(desiredAngle)));
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
