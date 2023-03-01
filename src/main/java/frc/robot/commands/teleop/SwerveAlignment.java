package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.photonvision.targeting.PhotonPipelineResult;

public class SwerveAlignment extends SequentialCommandGroup {
    public SwerveAlignment(Swerve swerve, Limelight limelight, PhotonVision photonVision, XboxController coController) {
        addCommands(
                new SwerveAlignmentX(swerve, limelight, photonVision),
                new SwerveAlignmentY(swerve, limelight, photonVision, coController)
        );
    }
}
