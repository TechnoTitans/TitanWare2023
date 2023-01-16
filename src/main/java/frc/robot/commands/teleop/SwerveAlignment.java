package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.wrappers.sensors.Limelight;

public class SwerveAlignment extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;

    public SwerveAlignment(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (limelight.isTargetFound()){
            swerve.drive(limelight.getSwerveError(),0,0,true);
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
