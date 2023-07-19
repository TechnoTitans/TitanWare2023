package frc.robot.commands.teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;
import frc.robot.utils.teleop.ControllerUtils;
import org.littletonrobotics.junction.Logger;

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
    public void initialize() {}

    @Override
    public void execute() {
        if (!elevator.verticalIsExtended()) {
            if (controller.getLeftTriggerAxis() > 0.5) {
                Profiler.setSwerveSpeed(Enums.SwerveSpeed.SLOW);
            } else if (controller.getRightTriggerAxis() > 0.5) {
                Profiler.setSwerveSpeed(Enums.SwerveSpeed.FAST);
            } else {
                Profiler.setSwerveSpeed(Enums.SwerveSpeed.NORMAL);
            }
        } else {
            if (controller.getRightTriggerAxis() > 0.5) {
                Profiler.setSwerveSpeed(Enums.SwerveSpeed.NORMAL);
            } else {
                Profiler.setSwerveSpeed(Enums.SwerveSpeed.SLOW);
            }
        }

        final Enums.DriverProfile driverProfile = Profiler.getDriverProfile();
        final Enums.SwerveSpeed swerveSpeed = Profiler.getSwerveSpeed();

        final double throttleWeight = swerveSpeed.getThrottleWeight();
        final double rotWeight = swerveSpeed.getRotateWeight();

        final double xSpeed = ControllerUtils.getStickInputWithWeight(
                controller.getLeftY(),
                0.01,
                Constants.Swerve.TELEOP_MAX_SPEED,
                driverProfile.getThrottleSensitivity(),
                throttleWeight
        );

        final double ySpeed = ControllerUtils.getStickInputWithWeight(
                controller.getLeftX(),
                0.01,
                Constants.Swerve.TELEOP_MAX_SPEED,
                driverProfile.getThrottleSensitivity(),
                throttleWeight
        );

        if (controller.getRightStickButton()) {
            final double angle = -Units.radiansToDegrees(
                    Math.atan2(-controller.getRightY(), controller.getRightX())
            ) + 90;

            swerve.faceDirection(
                    xSpeed,
                    ySpeed,
                    angle,
                    true,
                    1
            );
        } else {
            final double rot = ControllerUtils.getStickInputWithWeight(
                    controller.getRightX(),
                    0.01,
                    Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED,
                    driverProfile.getRotationalSensitivity(),
                    rotWeight
            );

            Logger.getInstance().recordOutput("Controller/XInput", xSpeed);
            Logger.getInstance().recordOutput("Controller/YInput", ySpeed);
            Logger.getInstance().recordOutput("Controller/RotationInput", rot);

            swerve.drive(
                    xSpeed,
                    ySpeed,
                    rot,
                    true
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Profiler.setSwerveSpeed(Enums.SwerveSpeed.NORMAL);
    }
}
