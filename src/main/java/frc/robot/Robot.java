// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.profiler.Profiler;
import frc.robot.utils.Enums;
import frc.robot.utils.TitanBoard;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        final Logger logger = Logger.getInstance();
        robotContainer = new RobotContainer();

        // record git metadata
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            // no need to inspect this here because BuildConstants is a dynamically changing file upon compilation
            //noinspection DataFlowIssue
            case 1:
                logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.CURRENT_MODE) {
            case REAL:
                // TODO: need log to USB or some other sort of data storage for real
                logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                // log to working directory when running sim
                logger.addDataReceiver(new WPILOGWriter(""));
                logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                final String logPath = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(logPath));
                logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        LiveWindow.disableAllTelemetry();

        SmartDashboard.putData("Field", robotContainer.field);

        TitanBoard.addDouble("Yaw", () -> robotContainer.swerve.getHeading() % 360);
        TitanBoard.addDouble("Pitch", robotContainer.swerve::getPitch);
        TitanBoard.addEncoder("EVertical Enc",
                () -> robotContainer.elevatorVerticalEncoder.getPosition().refresh().getValue(),
                () -> robotContainer.elevatorVerticalEncoder.getVelocity().refresh().getValue()
        );
        TitanBoard.addEncoder("EHorizontal Enc",
                robotContainer.elevatorHorizontalEncoder::getPosition, robotContainer.elevatorHorizontalEncoder::getVelocity
        );
        TitanBoard.addEncoder("Tilt Enc",
                robotContainer.clawTiltEncoder::getAbsolutePosition, robotContainer.clawTiltEncoder::getVelocity
        );
        TitanBoard.addEncoder("OpenClose Enc",
                robotContainer.clawOpenCloseEncoder::getAbsolutePosition, robotContainer.clawOpenCloseEncoder::getVelocity
        );

        TitanBoard.addBoolean("Robot Enabled", DriverStation::isEnabled);

        TitanBoard.addSwerveModuleStates("FL", robotContainer.frontLeft);
        TitanBoard.addSwerveModuleStates("FR", robotContainer.frontRight);
        TitanBoard.addSwerveModuleStates("BL", robotContainer.backLeft);
        TitanBoard.addSwerveModuleStates("BR", robotContainer.backRight);

        TitanBoard.addBoolean("Vertical Elevator LS", robotContainer.elevatorVerticalLimitSwitch::get);
        TitanBoard.addBoolean("Horizontal Elevator Back LS", robotContainer.elevatorHorizontalLimitSwitch::get);

        logger.start();
        TitanBoard.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        robotContainer.candleController.setState(Enums.CANdleState.OFF);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.photonApriltags.refreshAlliance();

        //Set Profile
        Profiler.setProfile(robotContainer.profileChooser.getSelected());

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        CommandScheduler.getInstance().setDefaultCommand(robotContainer.swerve, robotContainer.swerveDriveTeleop);
        robotContainer.elevatorTeleop.schedule();
        robotContainer.intakeTeleop.schedule();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        File[] paths = new File(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toString()).listFiles();
        if (paths == null) return;
        for (File path : paths) {
            boolean deleteSuccess = path.delete();
            DriverStation.reportWarning(String.format("File Delete %s", deleteSuccess ? "Success" : "Fail"), false);
        }
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
