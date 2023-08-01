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
import frc.robot.utils.logging.LogUtils;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class AutoAlignmentV3 extends CommandBase {
    private final Swerve swerve;
    private final Elevator elevator;
    private final Claw claw;
    private final PhotonVision photonVision;
    private final TrajectoryManager trajectoryManager;

    private SequentialCommandGroup sequentialCommandGroup;

    private final Translation2d leftPastCharge = new Translation2d(2.3, 4.7);
    private final Translation2d rightPastCharge = new Translation2d(2.3, 0.75);

    private boolean withLeftSide = true;

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
    public AutoAlignmentV3 withLeftSide(final boolean withLeftSide) {
        this.withLeftSide = withLeftSide;
        return this;
    }

    @Override
    public void initialize() {
        Logger.getInstance().recordOutput("AutoAlign/IsActive", true);

        this.sequentialCommandGroup = new SequentialCommandGroup();
        final Pose2d currentPose = photonVision.getEstimatedPosition();

        final List<PathPoint> trajectoryPathPoints = new ArrayList<>();

        trajectoryPathPoints.add(
                new PathPoint(
                        currentPose.getTranslation(),
                        Rotation2d.fromDegrees(180),
                        currentPose.getRotation()
                )
        );

        //TODO: Calculate scalar values
        if (withLeftSide) {
            final double distanceX = currentPose.getTranslation().minus(leftPastCharge).getX();
            final double distanceY = Math.abs(currentPose.getTranslation().minus(leftPastCharge).getY());
            final double distances = (9 / distanceX) + (distanceY * 0.9);
            if (distances <= 0) return;
            trajectoryPathPoints.add(
                    new PathPoint(
                            leftPastCharge,
                            Rotation2d.fromDegrees(180),
                            Rotation2d.fromDegrees(180)
                    ).withPrevControlLength(distances)
            );
        } else {
            final double distanceX = currentPose.getTranslation().minus(rightPastCharge).getX();
            final double distanceY = Math.abs(currentPose.getTranslation().minus(rightPastCharge).getY());
            final double distances = (9 / distanceX) + (distanceY * 0.9);
            if (distances <= 0) return;
            trajectoryPathPoints.add(
                    new PathPoint(
                            rightPastCharge,
                            Rotation2d.fromDegrees(180),
                            Rotation2d.fromDegrees(180)
                    ).withPrevControlLength(distances)
            );
        }

        final Rotation2d angle = currentPose.getTranslation().minus(trajectoryPathPoints.get(1).position).getAngle()
                .rotateBy(Rotation2d.fromDegrees(180));

        final ChassisSpeeds swerveChassisSpeeds = swerve.getFieldRelativeSpeeds();
        trajectoryPathPoints.set(
                0,
                new PathPoint(
                        currentPose.getTranslation(),
                        angle,
                        currentPose.getRotation(),
                        Math.hypot(swerveChassisSpeeds.vxMetersPerSecond, swerveChassisSpeeds.vyMetersPerSecond)
                )
        );

        final PathConstraints trajectoryConstrains = new PathConstraints(
                Constants.Swerve.TRAJECTORY_MAX_SPEED,
                Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
        );

        final PathPlannerTrajectory mainTrajectory = PathPlanner.generatePath(
                trajectoryConstrains,
                trajectoryPathPoints
        );

        Logger.getInstance().recordOutput(
                "AutoAlign/Trajectory", LogUtils.LoggableTrajectory.fromTrajectory(mainTrajectory));

        final Translation2d targetError = currentPose.getTranslation().minus(trajectoryPathPoints.get(1).position);
        final double hypot = Math.hypot(targetError.getX(), targetError.getY());

        if (!trajectoryPathPoints.isEmpty() && !robotInCommunity(currentPose) && hypot > 3) {
            this.sequentialCommandGroup.addCommands(trajectoryManager.getCommand(mainTrajectory));
        }

        final Pose2d targetPose = AlignmentZone.CENTER.getAlignmentPosition(
                AlignmentZone.GenericDesiredAlignmentPosition.LEFT);

        sequentialCommandGroup.addCommands(new LineUpToGrid(swerve, photonVision, targetPose));

        //Score Sequence
        sequentialCommandGroup.addCommands(
                Commands.runOnce(
                        () -> {
                            elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH);
                            claw.setDesiredState(Enums.ClawState.CLAW_DROP);
                        }
                ),
                Commands.waitSeconds(1.5),
                Commands.runOnce(() -> {
                    claw.setDesiredState(Enums.ClawState.CLAW_OUTTAKE);
                }),
                Commands.waitSeconds(0.7),
                Commands.runOnce(
                        () -> {
                            elevator.setDesiredState(Enums.ElevatorState.ELEVATOR_STANDBY);
                            claw.setDesiredState(Enums.ClawState.CLAW_STANDBY);
                        }
                ),
                Commands.waitSeconds(0.3)
        );

        sequentialCommandGroup.schedule();
    }

    @Override
    public boolean isFinished() {
        return sequentialCommandGroup.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        sequentialCommandGroup.cancel();
        Logger.getInstance().recordOutput("AutoAlign/IsActive", false);
    }

    private boolean robotInCommunity(final Pose2d currentPose) {
        return PoseUtils.poseWithinArea(
                currentPose,
                new Translation2d(1.15, 0),
                new Translation2d(3.5,  5.5),
                PoseUtils.MirroringBehavior.MIRROR_ACROSS_GRID_CENTER_POINT);
    }
}