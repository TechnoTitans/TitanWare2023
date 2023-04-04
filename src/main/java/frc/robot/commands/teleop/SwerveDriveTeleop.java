package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;

public class SwerveDriveTeleop extends CommandBase {
    private final Swerve swerve;
    private final Elevator elevator;
    private final XboxController controller;
    private Profiler driverProfile;


    public SwerveDriveTeleop(Swerve swerve, Elevator elevator, XboxController controller) {
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
        double frontBack = MathMethods.deadband(controller.getLeftY(), 0.01) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();
        double leftRight = MathMethods.deadband(controller.getLeftX(), 0.01) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();

        if (!elevator.verticalIsExtended()) {
            if (controller.getLeftTriggerAxis() > 0.5) {
                Profiler.setWeights(Enums.SwerveSpeeds.SLOW);
            } else if (controller.getRightTriggerAxis() > 0.5) {
                Profiler.setWeights(Enums.SwerveSpeeds.FAST);
            } else {
                Profiler.setWeights(Enums.SwerveSpeeds.NORMAL);
            }
        } else {
            Profiler.setWeights(Enums.SwerveSpeeds.SLOW);
        }

        double throttleWeight = driverProfile.getThrottleWeight();
        double rotWeight = driverProfile.getRotateWeight();

        if (controller.getRightStickButton()) {
            double angle = -Math.toDegrees(Math.atan2(-controller.getRightY(), controller.getRightX())) + 90;
            swerve.faceDirection(frontBack * throttleWeight, leftRight * throttleWeight, angle, true, 1);
        } else {
            double rot = MathMethods.deadband(controller.getRightX(), 0.01) * Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED * driverProfile.getRotationalSensitivity();
//            swerve.drive(frontBack * throttleWeight, leftRight * throttleWeight, rot * rotWeight, true);
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
