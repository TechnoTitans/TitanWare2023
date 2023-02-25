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
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("gyro", robotContainer.swerve.getHeading());
        SmartDashboard.putNumber("HE Enc", robotContainer.elevatorHorizontalNeo.getAlternateEncoder(8192).getPosition());
//        SmartDashboard.putNumber("Claw open close Encoder", robotContainer.claw.getOpenCloseEncPosition());
        SmartDashboard.putBoolean("button", robotContainer.elevatorHorizontalLimitSwitch.get());
        SmartDashboard.putNumber("Claw enc", robotContainer.clawOpenCloseEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Tilt enc", robotContainer.clawTiltNeo.getAlternateEncoder(8192).getPosition());
        SmartDashboard.putBoolean("Claw LS", robotContainer.clawTiltLimitSwitch.get());
        robotContainer.elevator.telemetry();

//        robotContainer.clawTiltNeo.coast();

//        SmartDashboard.putNumber("prox", robotContainer.clawColorSensor.getProximity());
//        SmartDashboard.putNumber("blue", robotContainer.clawColorSensor.getColor().blue);

//        robotContainer.claw.getClawTiltNeo().set(CANSparkMax.ControlType.kPosition, 0);
    }

    @Override
    public void disabledInit() {
        robotContainer.limeLight.setLEDMode(Enums.LimeLightLEDState.LED_ON);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.swerve.zeroRotation();
        robotContainer.swerve.brake();
        autonomousCommand = robotContainer.getAutonomousCommand();

//         //schedule the autonomous command (example)
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
        CommandScheduler.getInstance().schedule(robotContainer.elevatorTeleop);

        CommandScheduler.getInstance().schedule(robotContainer.intakeTeleop);
    }

    @Override
    public void teleopPeriodic() {

//        robotContainer.odometry.update(robotContainer.swerve.getRotation2d(), robotContainer.swerve.getModulePositions());
//        robotContainer.field.setRobotPose(robotContainer.odometry.getPoseMeters());

//        robotContainer.elevatorVerticalMotor.set(robotContainer.oi.getXboxCo().getLeftY());


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
        SmartDashboard.putNumber("Tilt enc", robotContainer.clawTiltNeo.getAlternateEncoder(8192).getPosition());
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
