package frc.robot.commands.teleop;

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
    boolean fieldRelative = true;


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
        double frontBack = MathMethods.deadband(controller.getLeftY(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();
        double leftRight = MathMethods.deadband(controller.getLeftX(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();

        double throttleWeight;
        double turnWeight;
        if (controller.getLeftTriggerAxis() > 0.5) {
            throttleWeight = 0.214; //0.3
            turnWeight = 0.357; //0.5
        } else if (controller.getRightTriggerAxis() > 0.5) {
            throttleWeight = 1;
            turnWeight = 1;
        } else {
            throttleWeight = 0.357; //0.5
            turnWeight = 0.7; //0.7
        }

        double rot = MathMethods.deadband(controller.getRightX(), 0.1) * Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED * driverProfile.getRotationalSensitivity();
        swerve.drive(frontBack * throttleWeight, leftRight * throttleWeight, rot * turnWeight, fieldRelative);
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
