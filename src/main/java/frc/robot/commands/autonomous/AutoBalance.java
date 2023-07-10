package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.MathUtils;

public class AutoBalance extends CommandBase {
    private final Swerve swerve;
    private final PIDController pitchPIDController;
    private final double p = 0.03;
    private boolean flag = false;

    public AutoBalance(final Swerve swerve) {
        this.swerve = swerve;
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
            pitchPIDController.setP(0.024);
            flag = true;
        }
        final double pidOutput = MathUtil.clamp(pitchPIDController.calculate(swerve.getPitch() + swerve.getRoll()), -1, 1);
        swerve.drive(
                pidOutput,
                0,
                0,
                true);
    }

    @Override
    public boolean isFinished() {
        return flag && MathUtils.withinRange(swerve.getPitch() + swerve.getRoll(), 0, 2);
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
