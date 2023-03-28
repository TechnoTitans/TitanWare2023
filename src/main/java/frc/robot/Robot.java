// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.profiler.Profiler;
import frc.robot.utils.Enums;
import org.photonvision.EstimatedRobotPose;

import java.io.File;
import java.util.Optional;

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

    private void updatePose() {
        robotContainer.poseEstimator.update(
                robotContainer.swerve.getRotation2d(),
                robotContainer.swerve.getModulePositions());

        Optional<EstimatedRobotPose> result = robotContainer.photonApriltagCam.getEstimatedGlobalPose(
                        robotContainer.poseEstimator.getEstimatedPosition());
        if (false && result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            robotContainer.poseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(),
                    camPose.timestampSeconds);
        }

        robotContainer.field.getObject("robot").setPose(robotContainer.poseEstimator.getEstimatedPosition());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        updatePose();
        SmartDashboard.putNumber("gyro", robotContainer.swerve.getHeading());
        SmartDashboard.putNumber("pitch", robotContainer.swerve.getPitch());
        SmartDashboard.putNumber("amp", robotContainer.frontLeftDrive.getStatorCurrent().getValue());
//        SmartDashboard.putNumber("dt current", robotContainer.swerve.);
    }

    @Override
    public void disabledInit() {
        robotContainer.limeLight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
        robotContainer.candleController.setState(Enums.CANdleState.OFF);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
//
//         schedule the autonomous command (example)
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
