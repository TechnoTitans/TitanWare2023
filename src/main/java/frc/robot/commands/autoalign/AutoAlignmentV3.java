package frc.robot.commands.autoalign;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.alignment.AlignmentZone;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.teleop.ElevatorClawCommand;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class AutoAlignmentV3 extends CommandBase {
    protected static final String logKey = "AutoAlign/";

    private final Swerve swerve;
    private final Elevator elevator;
    private final Claw claw;
    private final PhotonVision photonVision;
    private final TrajectoryManager trajectoryManager;

    private AlignmentZone.CommunitySide desiredCommunitySide;
    private SequentialCommandGroup commandGroup;

    private final Translation2d leftPastCharge = new Translation2d(5.76, 4.7);
    private final Translation2d rightPastCharge = new Translation2d(5.76, 0.75);

    public AutoAlignmentV3(
            final Swerve swerve,
            final Elevator elevator,
            final Claw claw,
            final PhotonVision photonVision,
            final TrajectoryManager trajectoryManager
    ) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.claw = claw;
        this.photonVision = photonVision;
        this.trajectoryManager = trajectoryManager;

        addRequirements(elevator, claw);
    }

    public AutoAlignmentV3 withDesiredCommunitySide(final AlignmentZone.CommunitySide communitySide) {
        this.desiredCommunitySide = communitySide;
        return this;
    }

    @Override
    public void initialize() {
        Logger.getInstance().recordOutput(logKey + "IsActive", true);

        if (desiredCommunitySide == null) {
            cancel();
            return;
        }

        final Pose2d currentPose = photonVision.getEstimatedPosition();
        final AlignmentZone currentAlignmentZone =
                AlignmentZone.getAlignmentZoneFromCurrentPose(currentPose);

        if (currentAlignmentZone == null) {
            cancel();
            return;
        }

        final SequentialCommandGroup sequentialCommandGroup = new SequentialCommandGroup();
        this.commandGroup = sequentialCommandGroup;

        if (!robotInCommunity(currentPose)) {
            final ChassisSpeeds swerveChassisSpeeds = swerve.getFieldRelativeSpeeds();
            final TitanTrajectory.Constraints constraints = TitanTrajectory.Constraints.getDefault();
            final TitanTrajectory trajectory = new TitanTrajectory.Builder()
                    .withConstraints(TitanTrajectory.Constraints.getDefault())
                    .add(currentPose, swerveChassisSpeeds)
                    .add(currentAlignmentZone.getTrajectoryTarget(desiredCommunitySide))
                    .withEndVelocityOverride(constraints.maxVelocity)
                    .build();

            Logger.getInstance().recordOutput(
                    logKey + "Trajectory",
                    LogUtils.LoggableTrajectory.fromTrajectory(trajectory)
            );

            sequentialCommandGroup.addCommands(trajectoryManager.getCommand(trajectory));
        }

        final Pose2d targetPose = AlignmentZone.CENTER.getAlignmentPosition(
                AlignmentZone.GenericDesiredAlignmentPosition.LEFT);

        //Score Sequence
        final ElevatorClawCommand elevatorClawCommand = new ElevatorClawCommand.Builder(elevator, claw)
                .withElevatorState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH)
                .wait(0.3)
                .withClawState(Enums.ClawState.CLAW_DROP)
                .wait(1.5)
                .withClawState(Enums.ClawState.CLAW_OUTTAKE)
                .wait(0.7)
                .withElevatorClawStates(Enums.ElevatorState.ELEVATOR_STANDBY, Enums.ClawState.CLAW_STANDBY)
                .build();

        sequentialCommandGroup.addCommands(new LineUpToGrid(swerve, photonVision, targetPose), elevatorClawCommand);
        sequentialCommandGroup.schedule();
    }

    @Override
    public boolean isFinished() {
        return commandGroup.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (commandGroup != null) {
            commandGroup.cancel();
        }

        Logger.getInstance().recordOutput(logKey + "IsActive", false);
    }

    private boolean robotInCommunity(final Pose2d currentPose) {
        return PoseUtils.poseWithinArea(
                currentPose,
                new Translation2d(1.15, 0),
                new Translation2d(3.5,  5.5),
                PoseUtils.MirroringBehavior.MIRROR_ACROSS_X_CENTER
        ) || PoseUtils.poseWithinArea(
                currentPose,
                new Translation2d(3.4, 0),
                new Translation2d(4.9, 4),
                PoseUtils.MirroringBehavior.MIRROR_ACROSS_X_CENTER
        );
    }
}