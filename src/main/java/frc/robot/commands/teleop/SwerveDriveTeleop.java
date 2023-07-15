package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;
import frc.robot.utils.MathUtils;

public class SwerveDriveTeleop extends CommandBase {
    private final Swerve swerve;
    private final Elevator elevator;
    private final XboxController controller;
    private Profiler driverProfile;

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
    public void initialize() {
        this.driverProfile = Profiler.getProfile();
    }

    @Override
    public void execute() {
        final double frontBack = -MathUtils.deadband(controller.getLeftY(), 0.01) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();
        final double leftRight = -MathUtils.deadband(controller.getLeftX(), 0.01) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();

        if (!elevator.verticalIsExtended()) {
            if (controller.getLeftTriggerAxis() > 0.5) {
                Profiler.setWeights(Enums.SwerveSpeeds.SLOW);
            } else if (controller.getRightTriggerAxis() > 0.5) {
                Profiler.setWeights(Enums.SwerveSpeeds.FAST);
            } else {
                Profiler.setWeights(Enums.SwerveSpeeds.NORMAL);
            }
        } else {
            if (controller.getRightTriggerAxis() > 0.5) {
                Profiler.setWeights(Enums.SwerveSpeeds.NORMAL);
            } else {
                Profiler.setWeights(Enums.SwerveSpeeds.SLOW);
            }
        }

        final double throttleWeight = driverProfile.getThrottleWeight();
        final double rotWeight = driverProfile.getRotateWeight();

        if (controller.getRightStickButton()) {
            final double angle = -Math.toDegrees(Math.atan2(-controller.getRightY(), controller.getRightX())) + 90;
            swerve.faceDirection(frontBack * throttleWeight, leftRight * throttleWeight, angle, true, 1);
        } else {
            final double rot = -MathUtils.deadband(controller.getRightX(), 0.01) * Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED * driverProfile.getRotationalSensitivity();
            swerve.drive(frontBack * throttleWeight, leftRight * throttleWeight, rot * rotWeight, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Profiler.setWeights(Enums.SwerveSpeeds.NORMAL);
    }
}
