package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class AutoBalance extends CommandBase {
    private final Swerve swerve;
    private final double holonomicRotation;
    private final PIDController pitchPIDController;
    private double lastError = 0;
    private double delta = 0;
    private final double p = 0.03;

    public AutoBalance(Swerve swerve, double holonomicRotation) {
        this.swerve = swerve;
        this.holonomicRotation = holonomicRotation;
        this.pitchPIDController = new PIDController(p, 0, 0);
        pitchPIDController.setTolerance(0.001);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pitchPIDController.setP(p);
        lastError = 0;
        delta = 0;
    }

    @Override
    public void execute() {
        if (swerve.getABSPitch() >= 9) {
            pitchPIDController.setP(p/2);
        }
        double pidOutput = MathUtil.clamp(pitchPIDController.calculate(swerve.getABSPitch()), -1, 1);

        lastError = swerve.getABSPitch();
        delta = (swerve.getABSPitch() - lastError);
        SmartDashboard.putNumber("delta", delta);
        swerve.drive(
                -pidOutput,
                0,
                0,
                true);
        SmartDashboard.putNumber("pidOutput", pidOutput);




    }

    @Override
    public boolean isFinished() {
        return delta <= -.3;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
        SmartDashboard.putBoolean("ENDED2",true);
    }
}
