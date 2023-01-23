// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.profiler.Profiler;
import frc.robot.utils.Enums;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        robotContainer.swerve.zeroRotation();
        robotContainer.limelight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
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
        //Set Profile
        Profiler.setProfile(robotContainer.profileChooser.getSelected());

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.swerve, robotContainer.swerveDriveTeleop);
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.odometry.update(robotContainer.swerve.getRotation2d(), robotContainer.swerve.getModulePositions());
        robotContainer.field.setRobotPose(robotContainer.odometry.getPoseMeters());
        SmartDashboard.putNumber("gyro", robotContainer.swerve.getHeading() % 360);
        SmartDashboard.putNumber("distance", robotContainer.limelight.calculateDistance());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.odometry.resetPosition(Rotation2d.fromDegrees(0), robotContainer.swerve.getModulePositions(), new Pose2d());
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
