package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.auto.AutoOption;
import frc.robot.utils.auto.DriveController;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.utils.control.DriveToPoseController;
import frc.robot.wrappers.sensors.vision.PhotonVision;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrajectoryManager {
    private final Swerve swerve;
    private final DriveController controller;
    private final DriveToPoseController holdPositionController;
    private final PhotonVision<?> photonVision;

    private final Claw claw;
    private final Elevator elevator;

    private final HashMap<AutoOption, List<TitanTrajectory>> cachedTitanTrajectories;

    public TrajectoryManager(
            final Swerve swerve,
            final DriveController controller,
            final DriveToPoseController holdPositionController,
            final PhotonVision<?> photonVision,
            final Claw claw,
            final Elevator elevator
    ) {
        this.swerve = swerve;
        this.controller = controller;
        this.holdPositionController = holdPositionController;
        this.photonVision = photonVision;

        this.claw = claw;
        this.elevator = elevator;

        this.cachedTitanTrajectories = new HashMap<>();
    }

    public void precomputeMarkerCommands(final List<AutoOption> autoOptions) {
        for (final AutoOption autoOption : autoOptions) {
            if (!cachedTitanTrajectories.containsKey(autoOption)) {
                cachedTitanTrajectories.put(autoOption, getTrajectoriesFromPath(autoOption));
            }
        }
    }

    public List<TitanTrajectory> getTrajectoriesFromPath(
            final List<AutoOption.PathNameWithConstraints> pathNameWithConstraintsList,
            final boolean reverseTrajectory
    ) {
        final TrajectoryFollower.FollowerContext followerContext =
                new TrajectoryFollower.FollowerContext(elevator, claw);

        return pathNameWithConstraintsList.stream()
                .map(
                        (pathNameWithConstraints) -> {
                            final TitanTrajectory.Constraints constraints = pathNameWithConstraints.constraints();
                            return TitanTrajectory.fromPathPlannerTrajectory(
                                    PathPlanner.loadPath(
                                            pathNameWithConstraints.name(),
                                            constraints.maxVelocity,
                                            constraints.maxAcceleration,
                                            reverseTrajectory
                                    ),
                                    followerContext
                            );
                        }
                )
                .toList();
    }

    public List<TitanTrajectory> getTrajectoriesFromPath(final AutoOption autoOption) {
        final List<TitanTrajectory> cachedTrajectories = cachedTitanTrajectories.get(autoOption);
        if (cachedTrajectories != null) {
            return cachedTrajectories;
        }

        return getTrajectoriesFromPath(
                autoOption.pathNameWithConstraintsList(), false
        );
    }

    public TrajectoryFollower getTrajectoryFollower(final TitanTrajectory trajectory) {
        // TODO: what happens if transformForAlliance is true?
        return new TrajectoryFollower(
                swerve,
                controller,
                holdPositionController,
                photonVision,
                trajectory,
                false,
                trajectory.getFollowerContext()
        );
    }

    public List<TrajectoryFollower> getTrajectoryFollowers(final AutoOption autoOption) {
        final List<TitanTrajectory> trajectories = getTrajectoriesFromPath(autoOption);
        return trajectories.stream()
                .map((trajectory) -> new TrajectoryFollower(
                        swerve,
                        controller,
                        holdPositionController,
                        photonVision,
                        trajectory,
                        true,
                        trajectory.getFollowerContext()
                ))
                .toList();
    }

    public SequentialCommandGroup getTrajectoryFollowerSequence(final AutoOption autoOption) {
        return new SequentialCommandGroup(
                getTrajectoryFollowers(autoOption).toArray(Command[]::new)
        );
    }

    @SuppressWarnings("unused")
    public void follow(final AutoOption autoOption) {
        getTrajectoryFollowerSequence(autoOption).schedule();
    }
}