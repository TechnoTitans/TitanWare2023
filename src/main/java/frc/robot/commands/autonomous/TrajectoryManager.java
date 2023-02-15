package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.DriveController;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;

import java.io.File;
import java.util.List;

@SuppressWarnings("unused")
public class TrajectoryManager {
    private final Swerve swerve;
    private final DriveController controller;
    private final SwerveDriveOdometry odometry;
    private final boolean reverseTrajectory = false;
    private final Field2d field;
    private SendableChooser<Command> autoChooser;

    private final Claw claw;

    public TrajectoryManager(Swerve swerve, DriveController controller, SwerveDriveOdometry odometry, Field2d field, Claw claw) {
        this.swerve = swerve;
        this.controller = controller;
        this.odometry = odometry;
        this.field = field;
        this.claw = claw;

        createChooser();
    }

    private void createChooser() {
        autoChooser = new SendableChooser<>();
        File[] paths = new File(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toString()).listFiles();
        if (paths == null) return;
        for (File path : paths) {
            String autoPath = path.getName().substring(0, path.getName().lastIndexOf("."));
            autoChooser.addOption(autoPath, this.getCommand(autoPath));
        }
        SmartDashboard.putData("Auton Chooser", autoChooser);
    }

    public Command getSelectedPath() {
        return autoChooser.getSelected();
    }

    public void follow(String trajDir, double maxVel, double maxAccl) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, maxVel, maxAccl, reverseTrajectory);
        TrajectroyFollower trajFollower = new TrajectroyFollower(swerve, controller, odometry, traj, field, claw);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public void follow(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        TrajectroyFollower trajFollower = new TrajectroyFollower(swerve, controller, odometry, traj, field, claw);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public TrajectroyFollower getCommand(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        return new TrajectroyFollower(swerve, controller, odometry, traj, field, claw);
    }

    public TrajectroyFollower getCommand(String trajDir, double maxVel, double maxAccl) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, maxVel, maxAccl, reverseTrajectory);
        return new TrajectroyFollower(swerve, controller, odometry, traj, field, claw);
    }
}

@SuppressWarnings("unused")
class TrajectroyFollower extends CommandBase {
    private final PathPlannerTrajectory traj;
    private final Timer timer;
    private final Swerve swerve;
    private final DriveController controller;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;
    private final Claw claw;

    public TrajectroyFollower(Swerve swerve, DriveController controller, SwerveDriveOdometry odometry, PathPlannerTrajectory traj, Field2d field, Claw claw) {
        this.swerve = swerve;
        this.timer = new Timer();
        this.controller = controller;
        this.odometry = odometry;
        this.traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
        this.field = field;

        this.claw = claw;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
        PathPlannerTrajectory.PathPlannerState initialState = traj.getInitialState();
        Pose2d initialPose = initialState.poseMeters;
        swerve.zeroRotation();
        odometry.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), initialPose);
        field.getObject("Traj").setPose(initialPose);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PathPlannerTrajectory.PathPlannerState sample = (PathPlannerTrajectory.PathPlannerState) traj.sample(currentTime);
        commander(sample);
        driveToState(sample);
        field.getObject("Traj").setPose(sample.poseMeters);
        odometry.update(swerve.getRotation2d(), swerve.getModulePositions());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        timer.stop();
        swerve.setAngle(traj.getEndState().holonomicRotation.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return !RobotState.isAutonomous() || timer.get() >= traj.getTotalTimeSeconds();
    }

    private void driveToState(PathPlannerTrajectory.PathPlannerState state) {
        ChassisSpeeds correction = controller.calculate(odometry.getPoseMeters(), state, swerve.getRotation2d());
        swerve.drive(correction);
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
                        case "testing":
                            SmartDashboard.putString("Testing", args[1]);
                            break;
                        case "claw":
                            claw.setState(Enums.ClawState.valueOf(args[1].toUpperCase()));
                            break;
                    }
                } catch (IllegalArgumentException e) {
//                    throw new IllegalArgumentException(e); //or this
                }
            }
        }
    }
}