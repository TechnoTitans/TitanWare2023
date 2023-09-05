package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;

@SuppressWarnings({"unused", "RedundantMethodOverride", "FieldCanBeLocal"})
public class AutoBalance extends CommandBase {
    //TODO: this all needs to be redone/done
    // do we even need AutoBalance? if so, it better be good
    private final Swerve swerve;

    public AutoBalance(final Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
