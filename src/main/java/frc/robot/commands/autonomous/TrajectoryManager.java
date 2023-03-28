package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.wrappers.sensors.vision.Limelight;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
public class TrajectoryManager {
    private final Swerve swerve;
    private final Field2d field2d;
    private final DriveController controller;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final boolean reverseTrajectory = false;
    private SendableChooser<Command> autoChooser;

    private final Claw claw;
    private final Elevator elevator;
    private final Limelight limelight;

    public TrajectoryManager(Swerve swerve, Field2d field2d, DriveController controller, SwerveDrivePoseEstimator poseEstimator, Claw claw, Elevator elevator, Limelight limelight) {
        this.swerve = swerve;
        this.field2d = field2d;
        this.controller = controller;
        this.poseEstimator = poseEstimator;

        this.claw = claw;
        this.elevator = elevator;
        this.limelight = limelight;

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
        TrajectoryFollower trajFollower = new TrajectoryFollower(swerve, field2d, controller, poseEstimator, traj, claw, elevator, limelight);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public void follow(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        TrajectoryFollower trajFollower = new TrajectoryFollower(swerve, field2d, controller, poseEstimator, traj, claw, elevator, limelight);
        CommandScheduler.getInstance().schedule(trajFollower);
    }

    public TrajectoryFollower getCommand(String trajDir) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, reverseTrajectory);
        return new TrajectoryFollower(swerve, field2d, controller, poseEstimator, traj, claw, elevator, limelight);
    }

    public TrajectoryFollower getCommand(String trajDir, double maxVel, double maxAccl) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(trajDir, maxVel, maxAccl, reverseTrajectory);
        return new TrajectoryFollower(swerve, field2d, controller, poseEstimator, traj, claw, elevator, limelight);
    }
}

@SuppressWarnings("unused")
class TrajectoryFollower extends CommandBase {
    private final PathPlannerTrajectory traj;
    private final Timer timer;

    private final Swerve swerve;
    private final Field2d field2d;
    private final DriveController controller;
    private final SwerveDrivePoseEstimator poseEstimator;
    private List<PathPlannerTrajectory.EventMarker> eventMarkers;

    private final Claw claw;
    private final Elevator elevator;
    private final Limelight limelight;

    private boolean paused = false;
    private boolean usingVision = false;
    private final PIDController xLimelightPIDController;

    public TrajectoryFollower(Swerve swerve, Field2d field2d, DriveController controller, SwerveDrivePoseEstimator poseEstimator, PathPlannerTrajectory traj, Claw claw, Elevator elevator, Limelight limelight) {
        this.swerve = swerve;
        this.timer = new Timer();
        this.field2d = field2d;
        this.controller = controller;
        this.poseEstimator = poseEstimator;
        this.traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

        this.claw = claw;
        this.elevator = elevator;
        this.limelight = limelight;

        this.xLimelightPIDController = new PIDController(0.1, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d initialPose = traj.getInitialHolonomicPose();
        swerve.setAngle(initialPose.getRotation().getDegrees());
        poseEstimator.resetPosition(initialPose.getRotation(), swerve.getModulePositions(), initialPose);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (!paused) {
            double currentTime = timer.get();
            PathPlannerTrajectory.PathPlannerState sample = (PathPlannerTrajectory.PathPlannerState) traj.sample(currentTime);
            commander(sample);
            if (usingVision) {
                driveToStateAlignment(sample);
            } else {
                driveToState(sample);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        timer.stop();
        swerve.setAngle(traj.getEndState().holonomicRotation.getDegrees());
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
    }

    @Override
    public boolean isFinished() {
        return !RobotState.isAutonomous();
    }

    private void driveToState(PathPlannerTrajectory.PathPlannerState state) {
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
        ChassisSpeeds correction = controller.calculate(poseEstimator.getEstimatedPosition(), state);

        field2d.getObject("wantedState").setPose(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation));

        swerve.faceDirection(
                -correction.vxMetersPerSecond,
                -correction.vyMetersPerSecond,
                -state.holonomicRotation.getDegrees(),
                true);
    }

    private void driveToStateAlignment(PathPlannerTrajectory.PathPlannerState state) {
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_ON);
        ChassisSpeeds correction = controller.calculate(poseEstimator.getEstimatedPosition(), state);
        swerve.faceDirection(
                -correction.vxMetersPerSecond,
                xLimelightPIDController.calculate(limelight.getX()),
                state.holonomicRotation.getDegrees(),
                true);
    }

    private void commander(PathPlannerTrajectory.PathPlannerState sample) {
        if (eventMarkers == null) {
            eventMarkers = traj.getMarkers();
        }
        if (eventMarkers == null || eventMarkers.size() == 0) {
            return;
        }
        PathPlannerTrajectory.EventMarker marker = eventMarkers.get(0);
        double distError = 0.1;
//        if (MathMethods.withinBand(marker.positionMeters.getDistance(sample.poseMeters.getTranslation()), distError)) {
        if (MathMethods.withinRange(marker.timeSeconds, timer.get(), distError)) {
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
                    case "vision":
                        sequentialCommands.add(new InstantCommand(() -> {
                            usingVision = Boolean.parseBoolean(args[1]);
                            if (usingVision) {
                                limelight.setPipeline(Enums.LimelightPipelines.valueOf(args[2].toUpperCase()));
                            }
                        }));
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
                    default:
                        break;
                }
            }
            //dtpause:true;wait:1;elevator:ELEVATOR_EXTENDED_HIGH;wait:1.5;claw:CLAW_DROP;wait:0.75;claw:CLAW_OUTTAKE;wait:0.75;claw:CLAW_STANDBY;elevator:ELEVATOR_STANDBY;dtpause:false;

//            dtpause:true;wait:1;elevator:ELEVATOR_EXTENDED_HIGH;wait:1.5;claw:CLAW_DROP;wait:0.75;claw:CLAW_OUTTAKE;wait:0.75;claw:CLAW_STANDBY;elevator:ELEVATOR_STANDBY;dtpause:false;
            new SequentialCommandGroup(sequentialCommands.toArray(new Command[0])).schedule();
        }

    }
}