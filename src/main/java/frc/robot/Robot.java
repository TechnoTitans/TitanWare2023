package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.profiler.Profiler;
import frc.robot.utils.Enums;
import frc.robot.utils.TitanBoard;
import frc.robot.utils.auto.PathPlannerUtil;
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
        final Logger logger = Logger.getInstance();
        robotContainer = new RobotContainer();

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

        // we practically never use LiveWindow, and apparently this causes loop overruns so disable it
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // schedule PathPlanner server to start
        if (Constants.PathPlanner.IS_USING_PATH_PLANNER_SERVER) {
            PathPlannerServer.startServer(Constants.PathPlanner.SERVER_PORT);
        }

        // disable joystick not found warnings when in sim
        DriverStation.silenceJoystickConnectionWarning(Constants.CURRENT_MODE == Constants.RobotMode.SIM);

        // record git metadata
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> logger.recordMetadata("GitDirty", "All changes committed");
            // no need to inspect this here because BuildConstants is a dynamically changing file upon compilation
            //noinspection DataFlowIssue
            case 1 -> logger.recordMetadata("GitDirty", "Uncommitted changes");
            default -> logger.recordMetadata("GitDirty", "Unknown");
        }

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                //TODO: figure out which port is occupied, use sda1 if sda2 is used
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

        SmartDashboard.putData("Field", robotContainer.field);

        TitanBoard.addDouble("Yaw",
                () -> MathUtil.inputModulus(robotContainer.swerve.getHeading(), 0, 360)
        );
        TitanBoard.addDouble("Pitch", robotContainer.swerve::getPitch);
        TitanBoard.addDouble("Roll", robotContainer.swerve::getRoll);

        TitanBoard.addBoolean("Robot Enabled", DriverStation::isEnabled);

        logger.start();
        TitanBoard.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().removeDefaultCommand(robotContainer.swerve);
        robotContainer.candleController.setState(Enums.CANdleState.OFF);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        //TODO: this causes loop overruns in sim, either check in real or address this issue
        // investigated and seems like Pigeon2.setYaw() is the culprit, address this eventually
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.photonApriltags.refreshAlliance();

        Profiler.setDriverProfile(robotContainer.profileChooser.getSelected());

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        CommandScheduler.getInstance().setDefaultCommand(robotContainer.swerve, robotContainer.swerveDriveTeleop);
        robotContainer.elevatorTeleop.schedule();
        robotContainer.intakeTeleop.schedule();
    }

    @Override
    public void teleopPeriodic() {}

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

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
