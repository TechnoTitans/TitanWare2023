package frc.robot.commands.autoalign;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.autonomous.TrajectoryFollower;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.commands.pathfinding.Node;
import frc.robot.commands.pathfinding.NodeField;
import frc.robot.commands.pathfinding.TranslationNode;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.utils.logging.LogUtils;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

public class AutoAlignmentV2 extends CommandBase {
    private final PhotonVision photonVision;
    private final TrajectoryManager trajectoryManager;
    private final NodeField nodeField;
    private final Pose2d goal;

    private TrajectoryFollower trajectoryFollower;

    public AutoAlignmentV2(
            final PhotonVision photonVision,
            final TrajectoryManager trajectoryManager,
            final NodeField nodeField,
            final Pose2d goal
    ) {
        this.photonVision = photonVision;
        this.trajectoryManager = trajectoryManager;
        this.nodeField = nodeField;
        this.goal = goal;
    }

    @Override
    public void initialize() {
        final Pose2d currentPose = photonVision.getEstimatedPosition();
        final Optional<List<Node<Translation2d>>> optionalNodes = nodeField.compute(
                currentPose.getTranslation(), goal.getTranslation()
        );

        if (optionalNodes.isEmpty()) {
            cancel();
            return;
        }

        final List<Node<Translation2d>> nodeList = optionalNodes.get();
        Logger.getInstance().recordOutput(
                "Path",
                nodeList.stream()
                        .map(translation2dNode ->
                                new Pose2d(translation2dNode.getValue(), new Rotation2d())
                        )
                        .toArray(Pose2d[]::new)
        );

        final PathConstraints pathConstraints = new PathConstraints(
                Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
        );

        final List<PathPoint> pathPoints = IntStream.range(0, nodeList.size())
                .mapToObj(i -> {
                    final Node<Translation2d> node = nodeList.get(i);
                    final Node<Translation2d> lastNode = (i > 0) ? nodeList.get(i - 1) : null;
                    final Node<Translation2d> nextNode = (i < (nodeList.size() - 1)) ? nodeList.get(i + 1) : null;

                    final Rotation2d heading;
                    if (lastNode == null && nextNode != null) {
                        final Translation2d diff = nextNode.getValue().minus(node.getValue());
                        heading = new Rotation2d(diff.getX(), diff.getY());
                    } else if (lastNode != null && nextNode == null) {
                        final Translation2d diff = node.getValue().minus(lastNode.getValue());
                        heading = new Rotation2d(diff.getX(), diff.getY());
                    } else if (nextNode != null) {
                        final Translation2d diff = nextNode.getValue().minus(node.getValue());
                        heading = new Rotation2d(diff.getX(), diff.getY());
                    } else {
                        heading = Rotation2d.fromDegrees(0);
                    }

                    return new PathPoint(
                            node.getValue(),
                            heading
                    );
                })
                .toList();

        final PathPlannerTrajectory trajectory = PathPlanner.generatePath(pathConstraints, pathPoints);
        Logger.getInstance().recordOutput(
                "Trajectory",
                LogUtils.LoggableTrajectory.fromTrajectory(trajectory)
        );

        this.trajectoryFollower = trajectoryManager.getCommand(TitanTrajectory.fromPathPlannerTrajectory(trajectory));
        this.trajectoryFollower.schedule();
    }

    @Override
    public boolean isFinished() {
        return trajectoryFollower == null || trajectoryFollower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (trajectoryFollower != null) {
            trajectoryFollower.cancel();
        }
    }
}
