package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.auto.AutoOption;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class TrajectoryManager {
    private final Swerve swerve;
    private final HolonomicPathFollowerConfig holonomicPathFollowerConfig;

    public TrajectoryManager(
            final Swerve swerve,
            final PhotonVision<?> photonVision
    ) {
        this.swerve = swerve;
        this.holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0),
                Constants.Swerve.MODULE_MAX_SPEED,
                Math.hypot(Constants.Swerve.WHEEL_BASE, Constants.Swerve.TRACK_WIDTH),
                new ReplanningConfig()
        );

        AutoBuilder.configureHolonomic(
                swerve::getEstimatedPosition,
                photonVision::resetPosition,
                swerve::getRobotRelativeSpeeds,
                swerve::drive,
                holonomicPathFollowerConfig,
                swerve
        );
    }

    public Command followAutoCommand(final AutoOption autoOption) {
        return new PathPlannerAuto(autoOption.name());
    }

    public Command followPathWithEventsCommand(final AutoOption.PathOption pathOption) {
        final PathPlannerPath path = PathPlannerPath.fromPathFile(pathOption.name());

        return new FollowPathWithEvents(
                new FollowPathHolonomic(
                        path,
                        swerve::getEstimatedPosition,
                        swerve::getRobotRelativeSpeeds,
                        swerve::drive,
                        holonomicPathFollowerConfig,
                        swerve
                ),
                path,
                swerve::getEstimatedPosition
        );
    }
}