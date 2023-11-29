package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Swerve;

public class AutoBalance extends Command {
    private final Swerve swerve;

    public AutoBalance(final Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.drive(2, 0, 0, false);
    }

    @Override
    public void execute() {
        final double pitchVelocity = swerve.getGyro().getFilteredPitchVelocity();

        //Needs to have pitched above certain degrees and has started to pitch down.
        if (swerve.getPitch().getDegrees() > 12 && pitchVelocity < -5) {
            swerve.wheelX();
            Commands.sequence(
                    Commands.waitSeconds(1),
                    Commands.runOnce(this::cancel)
            ).schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.wheelX();
    }
}
