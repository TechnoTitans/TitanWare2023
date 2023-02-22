package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class SwerveDriveTeleop extends CommandBase {
    private final Swerve swerve;
    private final XboxController controller;
    private Profiler driverProfile;

    public SwerveDriveTeleop(Swerve swerve, XboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.driverProfile = Profiler.getProfile();
    }

    @Override
    public void execute() {
        double throttleWeight = 1;
        double turnWeight = 1;

        if (controller.getRightTriggerAxis() > 0.5) {
            throttleWeight = 0.5;
            turnWeight = 0.5;
        }
        double frontBack = MathMethods.deadband(controller.getLeftY(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();
        double leftRight = MathMethods.deadband(controller.getLeftX(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();
        boolean fieldRelative = true;

        if (controller.getLeftBumper()) {
            swerve.faceDirection(frontBack, leftRight, 0, fieldRelative);
        } else if (controller.getRightBumper()) {
            swerve.faceDirection(frontBack, leftRight, 180, fieldRelative);
        } else {
            double rot = MathMethods.deadband(controller.getRightX(), 0.1) * Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED * driverProfile.getSpinningSensitivity();
            swerve.drive(frontBack * throttleWeight, leftRight * throttleWeight, rot * turnWeight, fieldRelative);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
