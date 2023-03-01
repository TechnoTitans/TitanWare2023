package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {

    Swerve swerve;
    double distance;
    int reverse;
    boolean flag = false;

    PIDController drivePID = new PIDController(.00009, 0, 0);
    double error;

    public AutoBalance(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
        swerve.resetDriveEncoders();
        flag = false;
//        swerve.getPigeon().reset();
    }

    @Override
    public void execute() {
        swerve.faceDirection(.5,  0, 180, true);
        if (swerve.getPigeon().getPitch().getValue() < -6) {
            flag = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (swerve.getPigeon().getPitch().getValue() > -6) && flag;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
