// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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
        robotContainer.swerve.brake();
        SmartDashboard.putData("Field", robotContainer.field);
        robotContainer.field.getObject("robot").setPose(robotContainer.poseEstimator.getEstimatedPosition());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("gyro", robotContainer.swerve.getHeading());
        SmartDashboard.putNumber("pitch", robotContainer.swerve.getPitch());
        SmartDashboard.putNumber("tiltenc", robotContainer.elevatorHorizontalEncoder.getPosition());

        if (robotContainer.elevator.getTargetState() == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH ||
                robotContainer.elevator.getTargetState() == Enums.ElevatorState.ELEVATOR_EXTENDED_MID) {
            System.out.println(DriverStation.getMatchType().toString() + "(" + DriverStation.getMatchTime() + "): " + robotContainer.poseEstimator.getEstimatedPosition());
        }
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

//        if (robotContainer.photonApriltagCam.robotOriginMatchesAlliance()) {
//            robotContainer.photonApriltagCam.loadTags();
//        }
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
        robotContainer.elevatorTeleop.schedule();
        robotContainer.intakeTeleop.schedule();
    }

    @Override
    public void teleopPeriodic() {
//        robotContainer.elevatorVerticalMotorMain.set(1);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        File[] paths = new File(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toString()).listFiles();
        if (paths == null) return;
        for (File path : paths) {
            path.delete();
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
