package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class LineUpToGrid extends CommandBase {
    private final Swerve swerve;
    private final PhotonVision photonVision;
    private final Pose2d targetPose;

    private final ProfiledPIDController alignPIDControllerX, alignPIDControllerY;

    public LineUpToGrid(
            final Swerve swerve,
            final PhotonVision photonVision,
            final Pose2d targetPose
    ) {
        this.swerve = swerve;
        this.photonVision = photonVision;
        this.targetPose = targetPose;

        this.alignPIDControllerX = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(4, 4)
        );
        this.alignPIDControllerX.setTolerance(0.1, 0.1);

        this.alignPIDControllerY = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(4, 4)
        );
        this.alignPIDControllerY.setTolerance(0.1, 0.1);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (targetPose == null) {
            cancel();
        }

        final ChassisSpeeds swerveChassisSpeeds = swerve.getFieldRelativeSpeeds();

        this.alignPIDControllerX.reset(
                photonVision.getEstimatedPosition().getX(), swerveChassisSpeeds.vxMetersPerSecond
        );

        this.alignPIDControllerY.reset(
                photonVision.getEstimatedPosition().getY(), swerveChassisSpeeds.vyMetersPerSecond
        );
    }

    @Override
    public void execute() {
        final Pose2d currentPose = photonVision.getEstimatedPosition();

        swerve.faceDirection(
                (Math.abs(alignPIDControllerY.getPositionError()) <= 0.05
                        || currentPose.getX() >= 2.75
                        ? alignPIDControllerX.calculate(currentPose.getX(), targetPose.getX())
                        : 0),
                (currentPose.getX() >= 2.75
                        ? 0
                        : alignPIDControllerY.calculate(currentPose.getY(), targetPose.getY())),
                targetPose.getRotation(),
                true,
                2
        );
    }

    @Override
    public boolean isFinished() {
        return alignPIDControllerX.atGoal() && alignPIDControllerY.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
