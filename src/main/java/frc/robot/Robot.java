// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
//        robotContainer.swerve.zeroRotation();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("gyro", robotContainer.swerve.getHeading());
        SmartDashboard.putNumber("pitch", robotContainer.swerve.getABSPitch());
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
        robotContainer.swerve.zeroRotation();
        robotContainer.swerve.brake();
        autonomousCommand = robotContainer.getAutonomousCommand();
//        robotContainer.swerve.resetDriveEncoders();
//
//         schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
//        robotContainer.swerve.faceDirection(0, 0, 60, true);
    }

    @Override
    public void teleopInit() {
        robotContainer.swerve.brake();
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

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.elevatorVerticalMotor.set(1);
//        robotContainer.odometry.resetPosition(Rotation2d.fromDegrees(0), robotContainer.swerve.getModulePositions(), new Pose2d());
//        if (robotContainer.oi.getXboxMain().getXButton()) {
//            File[] paths = new File(Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toString()).listFiles();
//            if (paths == null) return;
//            for (File path : paths) {
//                path.delete();
//            }
//        }

    }

    //TODO: remove this field
    double integratedMax = 0;

    @Override
    public void testPeriodic() {
        final double integratedRpm = Math.abs(
                (robotContainer.elevatorVerticalMotor.getSensorCollection().getIntegratedSensorVelocity()/2048)*10
        );

        if (integratedRpm > integratedMax)
            integratedMax = integratedRpm;
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
