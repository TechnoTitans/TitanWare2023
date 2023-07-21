package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.MathUtils;
import frc.robot.utils.alignment.AlignmentZone;
import frc.robot.utils.teleop.ControllerUtils;
import org.littletonrobotics.junction.Logger;

public class AutoAlignment extends CommandBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final XboxController mainController;
    private final ProfiledPIDController alignPIDController;

    private AlignmentZone.GenericDesiredAlignmentPosition desiredAlignmentPosition;
    private AlignmentZone desiredAlignmentZone;
    private Pose2d targetPose;

    public AutoAlignment(
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final XboxController mainController
    ) {
        this.swerve = swerve;
        this.mainController = mainController;
        this.poseEstimator = poseEstimator;
        this.alignPIDController = new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(4, 4)
        );
        this.alignPIDController.setTolerance(0.1, 0.1);

        addRequirements(swerve);
    }

    public AutoAlignment withDesiredAlignmentPosition(
            final AlignmentZone.GenericDesiredAlignmentPosition genericDesiredAlignmentPosition
    ) {
        this.desiredAlignmentPosition = genericDesiredAlignmentPosition;
        return this;
    }

    @Override
    public void initialize() {
        final ChassisSpeeds swerveChassisSpeeds = swerve.getRobotRelativeSpeeds();
        this.alignPIDController.reset(
                poseEstimator.getEstimatedPosition().getY()
        );

        if (desiredAlignmentPosition == null) {
            this.end(true);
            return;
        }

        final Pose2d currentPose = poseEstimator.getEstimatedPosition();
        final AlignmentZone currentAlignmentZone =
                AlignmentZone.getAlignmentZoneFromCurrentPose(currentPose);
        final AlignmentZone currentLoggedAlignmentZone =
                AlignmentZone.getAlignmentZoneFromCurrentPose(currentPose, true);

        if (currentAlignmentZone == null || currentLoggedAlignmentZone == null) {
            // if we're not in any alignment zone, then there's no point in doing anything so just ignore
            this.end(true);
            return;
        }

        desiredAlignmentZone = currentAlignmentZone;
        targetPose = currentAlignmentZone.getAlignmentPosition(desiredAlignmentPosition);

        Logger.getInstance().recordOutput("AutoAlign/IsActive", true);
        Logger.getInstance().recordOutput("AutoAlign/AlignmentZone", currentLoggedAlignmentZone.toString());
        Logger.getInstance().recordOutput("AutoAlign/WantedPose", targetPose);
        Logger.getInstance().recordOutput(
                "AutoAlign/AlignmentZoneTrajectory", currentLoggedAlignmentZone.getLoggablePoseRegionArray()
        );
    }

    @Override
    public void execute() {
        if (targetPose == null) {
            return;
        }

        final Pose2d currentPose = poseEstimator.getEstimatedPosition();
        final double xSpeed = ControllerUtils.getStickInputWithWeight(
                mainController.getLeftY(),
                0.01,
                Constants.Swerve.TELEOP_MAX_SPEED,
                Profiler.getDriverProfile().getThrottleSensitivity(),
                Profiler.getSwerveSpeed().getThrottleWeight()
        );

        final double controlEffort = alignPIDController.calculate(currentPose.getY(), targetPose.getY());
        swerve.faceDirection(
                xSpeed,
                alignPIDController.calculate(currentPose.getY(), targetPose.getY()),
                targetPose.getRotation().getDegrees(),
                true
        );

        Logger.getInstance().recordOutput("AutoAlign/ControlEffort", controlEffort);
        Logger.getInstance().recordOutput("AutoAlign/PositionError", alignPIDController.getPositionError());
        Logger.getInstance().recordOutput("AutoAlign/VelocityError", alignPIDController.getVelocityError());
    }

    @Override
    public boolean isFinished() {
        Logger.getInstance().recordOutput("AutoAlign/TargetPoseIsNull", targetPose == null);
        Logger.getInstance().recordOutput("AutoAlign/AtGoal", alignPIDController.atGoal());

        if (targetPose == null) {
            return true;
        }

        if (alignPIDController.atGoal()
                && MathUtils.withinTolerance(swerve.getYaw(), targetPose.getRotation().getRotations(), 0.1)
        ) {
            return true;
        }

        final Pose2d currentPose = poseEstimator.getEstimatedPosition();
        return !AlignmentZone.isPoseInAlignmentZone(currentPose, desiredAlignmentZone);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        targetPose = null;

        Logger.getInstance().recordOutput("AutoAlign/IsActive", false);
    }
}
