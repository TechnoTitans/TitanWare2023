// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.profiler.Profiler;
import frc.robot.utils.Enums;

import java.io.File;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        robotContainer.limeLight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("gyro", robotContainer.swerve.getHeading());
        SmartDashboard.putNumber("sm frontleft", robotContainer.frontLeftEncoder.getAbsolutePosition().getValue());
        SmartDashboard.putNumber("sm backleft", robotContainer.backLeftEncoder.getAbsolutePosition().getValue());
        SmartDashboard.putNumber("sm frontright", robotContainer.frontRightEncoder.getAbsolutePosition().getValue());
        SmartDashboard.putNumber("sm backright", robotContainer.backRightEncoder.getAbsolutePosition().getValue());
    }

    @Override
    public void disabledInit() {
        robotContainer.limeLight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.swerve.zeroRotation();
        robotContainer.swerve.brake();
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.swerve.coast();
        //Set Profile
        Profiler.setProfile(robotContainer.profileChooser.getSelected());

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.swerve, robotContainer.swerveDriveTeleop);
    }

    @Override
    public void teleopPeriodic() {
//        robotContainer.odometry.update(robotContainer.swerve.getRotation2d(), robotContainer.swerve.getModulePositions());
//        robotContainer.field.setRobotPose(robotContainer.odometry.getPoseMeters());

    }

    @SuppressWarnings("ResultOfMethodCallIgnored")
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.odometry.resetPosition(Rotation2d.fromDegrees(0), robotContainer.swerve.getModulePositions(), new Pose2d());
        if (robotContainer.oi.getXboxMain().getXButton()) {
            File[] paths = new File(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toString()).listFiles();
            if (paths == null) return;
            for (File path : paths) {
                path.delete();
            }
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
