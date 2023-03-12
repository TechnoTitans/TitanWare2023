package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class AutoBalance extends CommandBase {
    private final Swerve swerve;
    private final double holonomicRotation;
    private final PIDController pitchPIDController;

    private boolean hasLifted = false;
    private double flatLevel = 0;

    public AutoBalance(Swerve swerve, double holonomicRotation) {
        this.swerve = swerve;
        this.holonomicRotation = holonomicRotation;
        this.pitchPIDController = new PIDController(0.1, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        hasLifted = false;
        flatLevel = swerve.getABSPitch();
    }

    @Override
    public void execute() {
        swerve.faceDirection(
                pitchPIDController.calculate(16 - (swerve.getABSPitch(flatLevel))),
                0,
                holonomicRotation,
                true);

        if (!hasLifted && swerve.getABSPitch(flatLevel) > 5) {
            hasLifted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return hasLifted && MathMethods.withinRange(swerve.getABSPitch(flatLevel), 0, 1);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
