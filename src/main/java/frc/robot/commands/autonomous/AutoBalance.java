package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class AutoBalance extends CommandBase {
    private final Swerve swerve;
    private final double holonomicRotation;
    private final PIDController pitchPIDController;
    private final double p = 0.06;
    private boolean flag = false;

    public AutoBalance(Swerve swerve, double holonomicRotation) {
        this.swerve = swerve;
        this.holonomicRotation = holonomicRotation;
        this.pitchPIDController = new PIDController(p, 0, 0);
        pitchPIDController.setTolerance(0.001);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
        pitchPIDController.setP(p);
        flag = false;
    }

    @Override
    public void execute() {
        if ((swerve.getPitch() + swerve.getRoll()) >= 7.3) {
            pitchPIDController.setP(0.04/2);
            flag = true;
        }
        double pidOutput = MathUtil.clamp(pitchPIDController.calculate(swerve.getPitch() + swerve.getRoll()), -1, 1);
        swerve.drive(
                pidOutput,
                0,
                0,
                true);
    }

    @Override
    public boolean isFinished() {
        return flag && MathMethods.withinRange(swerve.getPitch() + swerve.getRoll(), 0, 2);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
    }
}
