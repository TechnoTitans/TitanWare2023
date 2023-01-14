package frc.robot.commands.teleop;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;

public class AutoBalanceTeleop extends CommandBase {
    private final Swerve swerve;
    private final Pigeon2 pigeon;

    public AutoBalanceTeleop(Swerve swerve, Pigeon2 pigeon2) {
        this.swerve = swerve;
        this.pigeon = pigeon2;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (!MathMethods.withinBand(swerve.getHeading(), 5)) {
            swerve.faceDirection(0, 0, 0, true);
        }
        double pitch = pigeon.getPitch();
        double val = pitch * Constants.Swerve.AUTO_BALANCE_PITCH_P;
        swerve.drive(val, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return MathMethods.withinBand(pigeon.getPitch(), 2);
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
