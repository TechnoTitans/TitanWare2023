package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class SwerveDriveTeleop extends CommandBase {
    private final Swerve swerve;
    private final XboxController controller;

    public SwerveDriveTeleop(Swerve swerve, XboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double frontBack = MathMethods.deadband(controller.getLeftY(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED;
        double leftRight = MathMethods.deadband(controller.getLeftX(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED;
        boolean fieldRelative = true;

        if (controller.getLeftBumper()) {
            swerve.faceDirection(frontBack, leftRight, 0, fieldRelative);
        } else if (controller.getRightBumper()) {
            swerve.faceDirection(frontBack, leftRight, 180, fieldRelative);
        } else if (controller.getBButtonPressed()) {
            swerve.faceClosest(frontBack, leftRight, fieldRelative);
        } else {
            double rot = MathMethods.deadband(controller.getRightX(), 0.1) * Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED;
            swerve.drive(frontBack, leftRight, rot, fieldRelative);
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
