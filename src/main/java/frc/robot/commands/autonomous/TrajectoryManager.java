package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.DriveController;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;

import java.io.File;
import java.util.ArrayList;
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
    private final Elevator elevator;

    public TrajectoryManager(Swerve swerve, DriveController controller, SwerveDriveOdometry odometry, Field2d field, Claw claw, Elevator elevator) {
        this.swerve = swerve;
        this.controller = controller;
        this.odometry = odometry;
        this.field = field;

        this.claw = claw;
        this.elevator = elevator;

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
        TrajectoryFollower trajFollower = new TrajectoryFollower(swerve, controller, odometry, traj, field, claw, elevator);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public void follow(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        TrajectoryFollower trajFollower = new TrajectoryFollower(swerve, controller, odometry, traj, field, claw, elevator);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public TrajectoryFollower getCommand(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        return new TrajectoryFollower(swerve, controller, odometry, traj, field, claw, elevator);
    }

    public TrajectoryFollower getCommand(String trajDir, double maxVel, double maxAccl) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, maxVel, maxAccl, reverseTrajectory);
        return new TrajectoryFollower(swerve, controller, odometry, traj, field, claw, elevator);
    }
}

@SuppressWarnings("unused")
class TrajectoryFollower extends CommandBase {
    private PathPlannerTrajectory traj;
    private PathPlannerTrajectory transformedTraj;
    private final Timer timer;

    private final Swerve swerve;
    private final DriveController controller;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;
    private List<PathPlannerTrajectory.EventMarker> eventMarkers;

    private final Claw claw;
    private final Elevator elevator;

    private boolean paused = false;

    public TrajectoryFollower(Swerve swerve, DriveController controller, SwerveDriveOdometry odometry, PathPlannerTrajectory traj, Field2d field, Claw claw, Elevator elevator) {
        this.swerve = swerve;
        this.timer = new Timer();
        this.controller = controller;
        this.odometry = odometry;
        this.traj = traj;
        this.transformedTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
        this.field = field;

        this.claw = claw;
        this.elevator = elevator;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // addRequirements(swerve); TODO IF AUTO DOESNT WORK TMR UNCOMMENT THIS
        PathPlannerTrajectory.PathPlannerState initialState = transformedTraj.getInitialState();
        Pose2d initialPose = initialState.poseMeters;
        swerve.setAngle(initialState.holonomicRotation.getDegrees());
        odometry.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), initialPose);
        field.getObject("Traj").setPose(initialPose);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (!paused) {
            double currentTime = timer.get();
            PathPlannerTrajectory.PathPlannerState sample = (PathPlannerTrajectory.PathPlannerState) transformedTraj.sample(currentTime);
            sample = PathPlannerTrajectory.transformStateForAlliance(sample, DriverStation.getAlliance());
            commander(sample);
            driveToState(sample);
            field.getObject("Traj").setPose(sample.poseMeters);
            odometry.update(swerve.getRotation2d(), swerve.getModulePositions());
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        timer.stop();
        swerve.setAngle(transformedTraj.getEndState().holonomicRotation.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return !RobotState.isAutonomous()
//                || timer.get() >= transformedTraj.getTotalTimeSeconds()
                ;
    }

    private void driveToState(PathPlannerTrajectory.PathPlannerState state) {
        ChassisSpeeds correction = controller.calculate(odometry.getPoseMeters(), state, swerve.getRotation2d());
        swerve.faceDirection(
                correction.vxMetersPerSecond,
                correction.vyMetersPerSecond,
                state.holonomicRotation.getDegrees(),
                true
        );
//        swerve.drive(correction);
    }

    private void commander(PathPlannerTrajectory.PathPlannerState sample) {
        if (eventMarkers == null) {
            eventMarkers = traj.getMarkers();
        }
        if (eventMarkers == null || eventMarkers.size() == 0) {
            return;
        }
        PathPlannerTrajectory.EventMarker marker = eventMarkers.get(0);
        double distError = 0.05;
        if (MathMethods.withinBand(marker.positionMeters.getDistance(sample.poseMeters.getTranslation()), distError)) {
            eventMarkers.remove(0);
            String[] commands = marker.names.get(0).trim().split(";");
            List<Command> sequentialCommands = new ArrayList<>();
            for (String x : commands) {
                String[] args = x.split(":");
                switch (args[0].toLowerCase()) {
                    case "claw":
                        sequentialCommands.add(new InstantCommand(() -> claw.setState(Enums.ClawState.valueOf(args[1].toUpperCase()))));
                        break;
                    case "elevator":
                        sequentialCommands.add(new InstantCommand(() -> elevator.setState(Enums.ElevatorState.valueOf(args[1].toUpperCase()))));
                        break;
                    case "autobalance":
                        sequentialCommands.add(new AutoBalance(swerve, sample.holonomicRotation.getDegrees()));
                        break;
                    case "wait":
                        sequentialCommands.add(new WaitCommand(Double.parseDouble(args[1])));
                        break;
                    case "dtpause":
                        sequentialCommands.add(new InstantCommand(() -> {
                            paused = Boolean.parseBoolean(args[1]);
                            if (paused) {
                                timer.stop();
                                swerve.stop();
                            } else {
                                timer.start();
                            }
                        }));
                        break;
                }
            }
//            dtpause:true;wait:1;elevator:ELEVATOR_EXTENDED_HIGH;wait:1.5;claw:CLAW_DROP;wait:0.75;claw:CLAW_OUTTAKE;wait:0.75;claw:CLAW_STANDBY;elevator:ELEVATOR_STANDBY;wait:1;dtpause:false;
            new SequentialCommandGroup(sequentialCommands.toArray(new Command[0])).schedule();
        }

    }
}