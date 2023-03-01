package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {

    private final Swerve swerve;
    private boolean flag = false;
    private double flatLevel = 0;
    private final double holonomicRotation;

//    PIDController drivePID = new PIDController(.00009, 0, 0);

    public AutoBalance(Swerve swerve, double holonomicRotation) {
        this.swerve = swerve;
        this.holonomicRotation = holonomicRotation;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
        flag = false;
        flatLevel = swerve.getPitch();
//        swerve.getPigeon().reset();
    }

    @Override
    public void execute() {
        swerve.faceDirection(.5,  0, holonomicRotation, true);
        if (swerve.getPitch() < (flatLevel - 6)) {
            flag = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (swerve.getPitch() > (flatLevel - 6)) && flag;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
