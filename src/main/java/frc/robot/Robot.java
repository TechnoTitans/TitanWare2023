package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.TitanBoard;
import frc.robot.utils.auto.PathPlannerUtil;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.ButtonBindings;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;
import java.util.List;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        if ((RobotBase.isReal() && Constants.CURRENT_MODE != Constants.RobotMode.REAL)
                || (RobotBase.isSimulation() && Constants.CURRENT_MODE == Constants.RobotMode.REAL)
        ) {
            DriverStation.reportWarning(String.format(
                    "Potentially incorrect CURRENT_MODE \"%s\" specified, robot is running \"%s\"",
                    Constants.CURRENT_MODE,
                    RobotBase.getRuntimeType().toString()
            ), true);

            throw new RuntimeException("Potentially incorrect CURRENT_MODE specified!");
        }

        final Logger logger = Logger.getInstance();

        // we practically never use LiveWindow, and apparently this causes loop overruns so disable it
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // schedule PathPlanner server to start
        if (Constants.PathPlanner.IS_USING_PATH_PLANNER_SERVER) {
            PathPlannerServer.startServer(Constants.PathPlanner.SERVER_PORT);
        }

        // register shutdown hook
        ToClose.hook();

        // disable joystick not found warnings when in sim
        DriverStation.silenceJoystickConnectionWarning(Constants.CURRENT_MODE == Constants.RobotMode.SIM);

        // record git metadata
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        // no need to inspect this here because BuildConstants is a dynamically changing file upon compilation
        //noinspection RedundantSuppression
        switch (BuildConstants.DIRTY) {
            //noinspection DataFlowIssue
            case 0 -> logger.recordMetadata("GitDirty", "All changes committed");
            //noinspection DataFlowIssue
            case 1 -> logger.recordMetadata("GitDirty", "Uncommitted changes");
            //noinspection DataFlowIssue
            default -> logger.recordMetadata("GitDirty", "Unknown");
        }

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                // TODO: this should be correct now, but we might need a more robust/permanent solution
                // figure out which port is occupied, use sda1 if sda is used
                // sda on Odin2023 is the CANivore, sda1 is open port
                logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
                logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                // log to working directory when running sim
                logger.addDataReceiver(new WPILOGWriter(""));
                logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                setUseTiming(false);
                final String logPath = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(logPath));
                logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        robotContainer = new RobotContainer();
        // precompute MarkerCommands from registered options on the auto chooser
        robotContainer.trajectoryManager.precomputeMarkerCommands(
                robotContainer.autoChooser.getRegisteredOptions()
        );

        TitanBoard.addDouble("Yaw", () -> GyroUtils.getAsAngleModdedDoubleDeg(robotContainer.swerve::getYaw));
        TitanBoard.addDouble("Pitch", () -> GyroUtils.getAsAngleModdedDoubleDeg(robotContainer.swerve::getPitch));
        TitanBoard.addDouble("Roll", () -> GyroUtils.getAsAngleModdedDoubleDeg(robotContainer.swerve::getRoll));

        TitanBoard.addBoolean("Robot Enabled", DriverStation::isEnabled);

        TitanBoard.start();
        logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        VirtualSubsystem.run();
    }

    @Override
    public void disabledInit() {
        robotContainer.swerve.setNeutralMode(NeutralModeValue.Brake);
//        robotContainer.elevator.setDesiredState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY);
//        robotContainer.claw.setDesiredState(SuperstructureStates.ClawState.CLAW_STANDBY);

        CommandScheduler.getInstance().removeDefaultCommand(robotContainer.swerve);
        CommandScheduler.getInstance().removeDefaultCommand(robotContainer.elevator);
        CommandScheduler.getInstance().removeDefaultCommand(robotContainer.claw);

        ButtonBindings.clear();

        CandleController.getInstance().setState(SuperstructureStates.CANdleState.OFF);
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        final PhotonVision<?> photonVision = robotContainer.photonVision;
        photonVision.refreshAlliance();

        robotContainer.swerve.setNeutralMode(NeutralModeValue.Coast);

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        robotContainer.elevator.setDesiredState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY);
        robotContainer.claw.setDesiredState(SuperstructureStates.ClawState.CLAW_STANDBY);
    }

    @Override
    public void teleopInit() {
        ButtonBindings.bindAll(robotContainer);

        final PhotonVision<?> photonVision = robotContainer.photonVision;
        photonVision.refreshAlliance();

        robotContainer.swerve.setNeutralMode(NeutralModeValue.Coast);

        Profiler.setDriverProfile(robotContainer.profileChooser.getSelected());

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        CommandScheduler.getInstance().setDefaultCommand(robotContainer.swerve, robotContainer.swerveDriveTeleop);
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.elevator, robotContainer.elevatorClawTeleop);
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.claw, robotContainer.elevatorClawTeleop);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        robotContainer.elevator.setDesiredState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY);
        robotContainer.claw.setDesiredState(SuperstructureStates.ClawState.CLAW_STANDBY);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        if (Constants.CURRENT_MODE == Constants.RobotMode.SIM) {
            // if we're in sim, don't delete the paths on the local computer
            return;
        }

        final List<File> paths = PathPlannerUtil.getAllPathPlannerPaths();
        for (final File path : paths) {
            final boolean deleteSuccess = path.delete();
            DriverStation.reportWarning(
                    String.format("File at %s deletion %s", path, deleteSuccess ? "Success" : "Failed"), false
            );
        }
    }

    @Override
    public void testPeriodic() {}
}
