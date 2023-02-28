package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.photonvision.targeting.PhotonPipelineResult;

public class SwerveAlignment extends SequentialCommandGroup {
    private Swerve swerve;
    private Limelight limelight;
    private PhotonVision photonVision;
    private XboxController coController;
    private Timer timer;
    private final PIDController xPhotonPIDController = new PIDController(0.5, 0.1, 0);
    private final PIDController yPhotonPIDController = new PIDController(2, 0.1, 0);
    private final PIDController xLimelightPIDController = new PIDController(0.1, 0, 0);
    private final PIDController yLimelightPIDController = new PIDController(.8, 0, 10);
    private final double DistanceOffset = -0.46;
    private final double LIMELIGHT_X_OFFSET = 0; //-14
    private double targetErrorX, targetErrorY;
    private Enums.VisionMode visionMode;
    double time = 0;

    boolean flag = false;

    private PhotonPipelineResult lastPipelineResult;

    public SwerveAlignment(Swerve swerve, Limelight limelight, PhotonVision photonVision, XboxController coController) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.photonVision = photonVision;
        this.coController = coController;
        this.timer = new Timer();

        addRequirements(swerve);


        addCommands(
                new SwerveAlignmentX(swerve, limelight, photonVision, coController),
                new SwerveAlignmentY(swerve, limelight, photonVision, coController)
        );
    }





}
