package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.teleop.ControllerUtils;

public class SwerveDriveTeleop extends CommandBase {
    private final Swerve swerve;
    private final Elevator elevator;
    private final XboxController controller;


    public SwerveDriveTeleop(
            final Swerve swerve,
            final Elevator elevator,
            final XboxController controller
    ) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        final double matchTime = DriverStation.getMatchTime();
        if (matchTime >= 0 && matchTime <= Constants.MATCH_END_THRESHOLD_SEC) {
            swerve.wheelX();
            return;
        }

        if (!elevator.verticalIsExtended()) {
            if (controller.getLeftTriggerAxis() > 0.5) {
                Profiler.setSwerveSpeed(Profiler.SwerveSpeed.SLOW);
            } else if (controller.getRightTriggerAxis() > 0.5) {
                Profiler.setSwerveSpeed(Profiler.SwerveSpeed.FAST);
            } else {
                Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL);
            }
        } else {
            if (controller.getRightTriggerAxis() > 0.5) {
                Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL);
            } else {
                Profiler.setSwerveSpeed(Profiler.SwerveSpeed.SLOW);
            }
        }

        final Profiler.DriverProfile driverProfile = Profiler.getDriverProfile();
        final Profiler.SwerveSpeed swerveSpeed = Profiler.getSwerveSpeed();

        final double throttleWeight = swerveSpeed.getThrottleWeight();
        final double rotWeight = swerveSpeed.getRotateWeight();

        final Translation2d leftStickSpeeds = ControllerUtils.getStickXYSquaredInput(
                controller.getLeftY(),
                controller.getLeftX(),
                0.01,
                Constants.Swerve.TELEOP_MAX_SPEED,
                driverProfile.getThrottleSensitivity(),
                throttleWeight
        );

        if (controller.getRightStickButton()) {
            final double rightStickXInput = ControllerUtils.applyDeadband(controller.getRightX(), 0.01);
            final double rightStickYInput = ControllerUtils.applyDeadband(controller.getRightY(), 0.01);

            final Rotation2d rightStickAngleDeg =
                    ControllerUtils.getFieldRelativeAngleFromStickInputs(rightStickXInput, rightStickYInput);

            swerve.faceDirection(
                    leftStickSpeeds.getX(),
                    leftStickSpeeds.getY(),
                    rightStickAngleDeg,
                    true
            );
        } else {
            final double rot = ControllerUtils.getStickSquaredInput(
                    controller.getRightX(),
                    0.01,
                    Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED,
                    driverProfile.getRotationalSensitivity(),
                    rotWeight
            );

            swerve.drive(
                    leftStickSpeeds.getX(),
                    leftStickSpeeds.getY(),
                    rot,
                    true
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Profiler.setSwerveSpeed(Profiler.SwerveSpeed.NORMAL);
    }
}
