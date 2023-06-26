// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.profiler.Profiler;
import frc.robot.utils.Enums;
import frc.robot.utils.TitanBoard;

import java.io.File;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

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

        TitanBoard.addEncoder("frontleft Enc",
                () -> robotContainer.frontLeftEncoder.getAbsolutePosition().refresh().getValue(),
                () -> robotContainer.frontLeftEncoder.getVelocity().refresh().getValue()
        );

        TitanBoard.addEncoder("frontright Enc",
                () -> robotContainer.frontRightEncoder.getAbsolutePosition().refresh().getValue(),
                () -> robotContainer.frontRightEncoder.getVelocity().refresh().getValue()
        );

        TitanBoard.addEncoder("backleft Enc",
                () -> robotContainer.backLeftEncoder.getAbsolutePosition().refresh().getValue(),
                () -> robotContainer.backLeftEncoder.getVelocity().refresh().getValue()
        );

        TitanBoard.addEncoder("backright Enc",
                () -> robotContainer.backRightEncoder.getAbsolutePosition().getValue(),
                () -> robotContainer.backRightEncoder.getVelocity().getValue()
        );

        TitanBoard.addBoolean("Vertical Elevator LS", robotContainer.elevatorVerticalLimitSwitch::get);
        TitanBoard.addBoolean("Horizontal Elevator Back LS", robotContainer.elevatorHorizontalLimitSwitch::get);

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
