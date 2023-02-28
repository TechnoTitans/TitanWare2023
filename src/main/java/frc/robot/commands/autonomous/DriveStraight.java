package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class DriveStraight extends CommandBase {

    Swerve swerve;
    double distance;
    int reverse;

    PIDController drivePID = new PIDController(.00009, 0, 0);
    double error;

    public DriveStraight(Swerve swerve, double distance, int reverse) {
        this.swerve = swerve;
        this.distance = distance;
        this.reverse = reverse;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetDriveEncoders();
    }

    @Override
    public void execute() {
        error = distance - swerve.getAvgEncoderValue();
        SmartDashboard.putNumber("drive err PID", -drivePID.calculate(error));
        SmartDashboard.putNumber("drive err", error);
        swerve.faceDirection(reverse * Math.min(-drivePID.calculate(error), 1.5), 0, 180, true);
    }

    @Override
    public boolean isFinished() {
        return MathMethods.withinBand(error, 500);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
