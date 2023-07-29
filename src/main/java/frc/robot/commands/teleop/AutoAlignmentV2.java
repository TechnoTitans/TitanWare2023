package frc.robot.commands.teleop;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.alignment.AlignmentZone;
import frc.robot.utils.sim.SimUtils;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class AutoAlignmentV2 extends CommandBase {
    private final Swerve swerve;
    private final PhotonVision photonVision;
    private final TrajectoryManager trajectoryManager;

    private SequentialCommandGroup sequentialCommandGroup;

    private final Translation2d leftBeforeCharge = new Translation2d(5.1, 4.7);
    private final Translation2d leftPastCharge = new Translation2d(2.3, 4.7);
    private final Translation2d rightBeforeCharge = new Translation2d(5.1, 0.75);
    private final Translation2d rightPastCharge = new Translation2d(2.3, 0.75);

    private boolean withLeftSide = true;

    public AutoAlignmentV2(
            final Swerve swerve,
            final PhotonVision photonVision,
            final TrajectoryManager trajectoryManager
    ) {
        this.swerve = swerve;
        this.photonVision = photonVision;
        this.trajectoryManager = trajectoryManager;
    }

    public AutoAlignmentV2 withLeftSide(final boolean withLeftSide) {
        this.withLeftSide = withLeftSide;
        return this;
    }

    private Rotation2d getAngleFromPoints(final Translation2d currentPose, final Translation2d targetPose) {
        return Rotation2d.fromRadians(
                Math.atan2(
                        targetPose.getY() - currentPose.getY(),
                        targetPose.getX() - currentPose.getX()
                )
        );
    }

    @Override
    public void initialize() {
        Logger.getInstance().recordOutput("AutoAlign/IsActive", true);

        this.sequentialCommandGroup = new SequentialCommandGroup();
        final Pose2d currentPose = photonVision.getEstimatedPosition();


        gridAlignment(currentPose);
    }

    @Override
    public void end(boolean interrupted) {
        sequentialCommandGroup.cancel();
        Logger.getInstance().recordOutput("AutoAlign/IsActive", false);
    }
    private boolean isWithinCommunity(final Pose2d currentPose) {
        return currentPose.getX() < 3.6 && currentPose.getY() < 5.5;
    }

    private void removeIfPast(final List<PathPoint> trajectoryPathPoints, final Pose2d currentPose) {
        if (!trajectoryPathPoints.isEmpty()) {
            final PathPoint lastPoint = trajectoryPathPoints.get(trajectoryPathPoints.size() - 1);
            if (currentPose.getX() < lastPoint.position.getX()) {
                trajectoryPathPoints.remove(lastPoint);
            }
        }
    }

    private void gridAlignment(final Pose2d currentPose) {
        final List<PathPoint> trajectoryPathPoints = new ArrayList<>();

        final boolean isWithinCommunity = isWithinCommunity(currentPose);
        if (!isWithinCommunity) {
            final ChassisSpeeds swerveChassisSpeeds = swerve.getFieldRelativeSpeeds();

            //Add Starting Position
            trajectoryPathPoints.add(
                    new PathPoint(
                            currentPose.getTranslation(),
                            Rotation2d.fromDegrees(180),
                            currentPose.getRotation()
                    )
            );

            //Avoid hitting things
            if (withLeftSide) {
                trajectoryPathPoints.add(
                        new PathPoint(
                                leftBeforeCharge,
                                Rotation2d.fromDegrees(-180),
                                Rotation2d.fromDegrees(180),
                                3
                        ).withNextControlLength(0.4)
                );
                removeIfPast(trajectoryPathPoints, currentPose);

                trajectoryPathPoints.add(
                        new PathPoint(
                                leftPastCharge,
                                Rotation2d.fromDegrees(180),
                                Rotation2d.fromDegrees(180),
                                3
                        ).withControlLengths(3, 0.2)
                );
                removeIfPast(trajectoryPathPoints, currentPose);

            } else {
                trajectoryPathPoints.add(
                        new PathPoint(
                                rightBeforeCharge,
                                Rotation2d.fromDegrees(-180),
                                Rotation2d.fromDegrees(180),
                                3
                        ).withNextControlLength(0.4)
                );
                removeIfPast(trajectoryPathPoints, currentPose);

                trajectoryPathPoints.add(
                        new PathPoint(
                                rightPastCharge,
                                Rotation2d.fromDegrees(180),
                                Rotation2d.fromDegrees(180),
                                3
                        ).withControlLengths(3, 0.2)
                );
                removeIfPast(trajectoryPathPoints, currentPose);
            }

            //Smooth first one
            trajectoryPathPoints.set(
                    0,
                    new PathPoint(
                            currentPose.getTranslation(),
                            getAngleFromPoints(currentPose.getTranslation(), trajectoryPathPoints.get(1).position),
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
                    "AutoAlign/Trajectory", SimUtils.LoggableTrajectory.fromTrajectory(mainTrajectory));

            if (!trajectoryPathPoints.isEmpty()) {
                sequentialCommandGroup.addCommands(trajectoryManager.getCommand(mainTrajectory));
            }
        }

        final Pose2d targetPose = AlignmentZone.CENTER.getAlignmentPosition(
                AlignmentZone.GenericDesiredAlignmentPosition.LEFT);
        sequentialCommandGroup.addCommands(new LineUpToGrid(swerve, photonVision, targetPose));

        sequentialCommandGroup.schedule();
    }
}
