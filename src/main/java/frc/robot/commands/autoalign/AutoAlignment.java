package frc.robot.commands.autoalign;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.MathUtils;
import frc.robot.utils.alignment.AlignmentZone;
import frc.robot.utils.teleop.ControllerUtils;
import org.littletonrobotics.junction.Logger;

public class AutoAlignment extends Command {
    private final Swerve swerve;
    private final XboxController mainController;
    private final ProfiledPIDController alignPIDController;
    private final String logKey = "AutoAlign";

    private AlignmentZone.GenericDesiredAlignmentPosition desiredAlignmentPosition;
    private AlignmentZone desiredUnmappedAlignmentZone;
    private Pose2d targetPose;

    public AutoAlignment(
            final Swerve swerve,
            final XboxController mainController
    ) {
        this.swerve = swerve;
        this.mainController = mainController;
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
        final ChassisSpeeds swerveChassisSpeeds = swerve.getFieldRelativeSpeeds();
        this.alignPIDController.reset(
                swerve.getEstimatedPosition().getY(), swerveChassisSpeeds.vyMetersPerSecond
        );

        if (desiredAlignmentPosition == null) {
            cancel();
            return;
        }

        final Pose2d currentPose = swerve.getEstimatedPosition();
        final AlignmentZone currentAlignmentZone =
                AlignmentZone.getAlignmentZoneFromCurrentPose(currentPose);
        final AlignmentZone currentUnmappedAlignmentZone =
                AlignmentZone.getAlignmentZoneFromCurrentPose(currentPose, true);

        if (currentAlignmentZone == null || currentUnmappedAlignmentZone == null) {
            // if we're not in any alignment zone, then there's no point in doing anything so just ignore
            cancel();
            return;
        }

        desiredUnmappedAlignmentZone = currentUnmappedAlignmentZone;
        targetPose = currentAlignmentZone.getAlignmentPosition(desiredAlignmentPosition);

        Logger.getInstance().recordOutput(logKey + "/IsActive", true);
        Logger.getInstance().recordOutput(logKey + "/DesiredAlignmentZone", desiredUnmappedAlignmentZone.toString());
        Logger.getInstance().recordOutput(logKey + "/WantedPose", targetPose);
        Logger.getInstance().recordOutput(
                logKey + "/AlignmentZoneTrajectory", desiredUnmappedAlignmentZone.getLoggablePoseRegionArray()
        );
    }

    @Override
    public void execute() {
        if (targetPose == null) {
            return;
        }

        final Pose2d currentPose = swerve.getEstimatedPosition();
        final double xSpeed = ControllerUtils.getStickSquaredInput(
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
                targetPose.getRotation(),
                true
        );

        Logger.getInstance().recordOutput(logKey + "/ControlEffort", controlEffort);
        Logger.getInstance().recordOutput(logKey + "/PositionError", alignPIDController.getPositionError());
        Logger.getInstance().recordOutput(logKey + "/VelocityError", alignPIDController.getVelocityError());
    }

    @Override
    public boolean isFinished() {
        final boolean targetPoseIsNull = targetPose == null;
        final boolean isAtGoal = alignPIDController.atGoal();

        Logger.getInstance().recordOutput(logKey + "/TargetPoseIsNull", targetPoseIsNull);
        Logger.getInstance().recordOutput(logKey + "/AlignPIDAtGoal", isAtGoal);

        // check targetPose first as it should be the least expensive check
        if (targetPoseIsNull) {
            return true;
        }

        // check the pid and rotation after targetPose check so that we aren't unnecessarily calling these
        // they should remain mostly inexpensive to call though, still
        if (isAtGoal
                && MathUtils.withinTolerance(
                        swerve.getYaw().getRadians(), targetPose.getRotation().getRadians(), Units.degreesToRadians(5))
        ) {
            return true;
        }

        // check this last so that we're only checking the pose against the alignment zone if we absolutely need to
        final Pose2d currentPose = swerve.getEstimatedPosition();
        return !AlignmentZone.isPoseInAlignmentZone(
                currentPose,
                desiredUnmappedAlignmentZone
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        targetPose = null;

        Logger.getInstance().recordOutput(logKey + "/IsActive", false);
    }
}
