package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

import java.util.List;

@SuppressWarnings("unused")
public class TrajectoryManager {
    private final Swerve swerve;
    private final HolonomicDriveController controller;
    private final SwerveDriveOdometry odometry;
    private final boolean reverseTrajectory = false;
    private final Field2d field;

    public TrajectoryManager(Swerve swerve, HolonomicDriveController controller, SwerveDriveOdometry odometry, Field2d field) {
        this.swerve = swerve;
        this.controller = controller;
        this.odometry = odometry;
        this.field = field;
    }

    public void follow(String trajDir, double periodic, double maxVel, double maxAccl) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, maxVel, maxAccl, reverseTrajectory);
        TrajFollower trajFollower = new TrajFollower(swerve, controller, odometry, traj, field);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public void follow(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        TrajFollower trajFollower = new TrajFollower(swerve, controller, odometry, traj, field);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public TrajFollower getCommand(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        return new TrajFollower(swerve, controller, odometry, traj, field);
    }

    public TrajFollower getCommand(String trajDir, double periodic, double maxVel, double maxAccl) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, maxVel, maxAccl, reverseTrajectory);
        return new TrajFollower(swerve, controller, odometry, traj, field);
    }

    public void testHolonomic(Pose2d targetPose, double velocity, Rotation2d targetRot) {
        swerve.drive(controller.calculate(odometry.getPoseMeters(), targetPose, velocity, targetRot));
    }
}

@SuppressWarnings("unused")
class TrajFollower extends CommandBase {
    private final PathPlannerTrajectory traj;
    private final Timer timer = new Timer();
    private final Swerve swerve;
    private final HolonomicDriveController controller;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    public TrajFollower(Swerve swerve, HolonomicDriveController controller, SwerveDriveOdometry odometry, PathPlannerTrajectory traj, Field2d field) {
        this.swerve = swerve;
        this.controller = controller;
        this.odometry = odometry;
        this.traj = traj;
        this.field = field;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
        PathPlannerTrajectory.PathPlannerState initialState = traj.getInitialState();
        Pose2d initialPose = initialState.poseMeters;
        swerve.setAngle(initialPose.getRotation().getDegrees()); //i added this not sure if it is good to have or not
        odometry.resetPosition(initialPose.getRotation(), swerve.getModulePositions(), initialPose);
        field.getObject("Traj").setPose(initialPose);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PathPlannerTrajectory.PathPlannerState sample = (PathPlannerTrajectory.PathPlannerState) traj.sample(currentTime);
//        commander(sample);
        driveToState(sample);
        field.getObject("Traj").setPose(sample.poseMeters);
        odometry.update(swerve.getRotation2d(), swerve.getModulePositions());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return !RobotState.isAutonomous() && timer.get() >= traj.getTotalTimeSeconds() - 1;
    }

    private void driveToState(PathPlannerTrajectory.PathPlannerState state) {
        ChassisSpeeds correction = controller.calculate(
                odometry.getPoseMeters(),
                state.poseMeters,
                state.velocityMetersPerSecond,
                state.holonomicRotation);
        //TODO: TEST FIELD RELATIVE PATH PLANNING AT DE
        swerve.faceDirection(correction.vxMetersPerSecond, correction.vyMetersPerSecond, state.holonomicRotation.getDegrees(), false);
    }

    private void commander(PathPlannerTrajectory.PathPlannerState sample) {
        List<PathPlannerTrajectory.EventMarker> eventMarkers = traj.getMarkers();
        if (eventMarkers.size() == 0) return;
        PathPlannerTrajectory.EventMarker marker = eventMarkers.get(0);
        double distError = 0.05; //TODO: tune this
        if (MathMethods.withinBand(marker.positionMeters.getDistance(sample.poseMeters.getTranslation()), distError)) {
            eventMarkers.remove(0);
            List<String> commands = marker.names;
            for (String x : commands) {
                String[] args = x.split(":");
                try {
                    switch (args[0].toLowerCase()) {
                        // TODO: CALL SUBSYSTEMS HERE
                    }
                } catch (IllegalArgumentException e) {
//                    throw new IllegalArgumentException(e); //or this
                }
            }
        }
    }
}