package frc.robot.commands.teleop;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class AutoBalanceTeleop extends CommandBase {
    private final Swerve swerve;
    private final Pigeon2 pigeon;
    private double pitch = 0;

    public AutoBalanceTeleop(Swerve swerve, Pigeon2 pigeon2) {
        this.swerve = swerve;
        this.pigeon = pigeon2;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        pitch = pigeon.getPitch().getValue();
        double val = pitch * Constants.Swerve.AUTO_BALANCE_PITCH_P;
        swerve.faceDirection(val, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return MathMethods.withinBand(pitch, 2);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
        // This should make an X pattern with the wheels
    }
}
