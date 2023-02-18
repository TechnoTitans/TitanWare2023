package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class SwerveAlignment extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;
    private final PhotonVision photonVision;
    private final XboxController coController;
    private final Timer timer;
    private final PIDController xPhotonPIDController = new PIDController(2, 0.1, 0);
    private final PIDController yPhotonPIDController = new PIDController(3, 0.1, 0);
    private final PIDController xLimelightPIDController = new PIDController(0.075, 0.1, 0);
    private final PIDController yLimelightPIDController = new PIDController(0.075, 0.1, 0);
    private final double PHOTON_X_OFFSET = -0.46;
    private final double LIMELIGHT_X_OFFSET = -0.5;
    private double targetErrorX, targetErrorY;
    private Enums.VisionMode visionMode;

    public SwerveAlignment(Swerve swerve, Limelight limelight, PhotonVision photonVision, XboxController coController) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.photonVision = photonVision;
        this.coController = coController;
        this.timer = new Timer();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        limelight.setLEDMode(Enums.LimeLightLEDState.LED_CONFIG);
        if (!photonVision.hasTargets() && !limelight.isTargetFound()) {
            end(true);
        } else if (photonVision.hasTargets() && limelight.isTargetFound()) {
            if (limelight.getY() > photonVision.getRobotPoseRelativeToAprilTag().getY()) {
                visionMode = Enums.VisionMode.PHOTON_VISION;
            } else {
                visionMode = Enums.VisionMode.LIME_LIGHT;
            }
        } else if (limelight.isTargetFound()) {
            visionMode = Enums.VisionMode.LIME_LIGHT;
        } else if (photonVision.hasTargets()) {
            visionMode = Enums.VisionMode.PHOTON_VISION;
        } else {
            end(true);
        }
    }

    @Override
    public void execute() {
//        if (!photonVision.hasTargets() && !limelight.isTargetFound()) {
//            end(true);
//        }

        if (visionMode == Enums.VisionMode.PHOTON_VISION) {
            final Pose2d targetPose = photonVision.getRobotPoseRelativeToAprilTag();

            targetErrorY = targetPose.getY();
            targetErrorX = targetPose.getX() + PHOTON_X_OFFSET;

            if (photonVision.targetId() == 6) { //Offset for substation
                targetErrorY += 0.5;
            }

            swerve.faceDirection(
                    xPhotonPIDController.calculate(targetErrorX),
                    yPhotonPIDController.calculate(targetErrorY),
                    0,
                    false
            );

        } else if (visionMode == Enums.VisionMode.LIME_LIGHT) {
            targetErrorY = limelight.calculateDistance();
            targetErrorX = limelight.getX();


            swerve.faceDirection(
                    yLimelightPIDController.calculate(targetErrorY),
                    xLimelightPIDController.calculate(-targetErrorX),
                    0,
                    false
            );
        }

        SmartDashboard.putNumber("LL X", targetErrorX);
        SmartDashboard.putNumber("LL Y", targetErrorY);

    }

    @Override
    public boolean isFinished() {
        return (MathMethods.withinBand(targetErrorY, 0.05) && MathMethods.withinBand(targetErrorX, 0.1)) ||
                timer.hasElapsed(5);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
        timer.stop();
        new WaitCommand(0.3) { //Rumble codriver control so they know that alignment has finished
            @Override
            public void initialize() {
                coController.setRumble(XboxController.RumbleType.kBothRumble, 0.5);
            }

            @Override
            public void end(boolean interrupted) {
                coController.setRumble(XboxController.RumbleType.kBothRumble, 0);
            }
        }.schedule();
    }
}
