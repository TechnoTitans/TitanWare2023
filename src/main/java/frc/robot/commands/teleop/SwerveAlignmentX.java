package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.photonvision.targeting.PhotonPipelineResult;

public class SwerveAlignmentX extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;
    private final PhotonVision photonVision;
    private final Timer timer;
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

    public SwerveAlignmentX(Swerve swerve, Limelight limelight, PhotonVision photonVision) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.photonVision = photonVision;
        this.timer = new Timer();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        flag = false;
        time = 0;

//        lastPipelineResult = photonVision.getLatestResult();
//
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_ON);
//        if (!photonVision.hasTargets(lastPipelineResult) && !limelight.isTargetFound()) {
//            end(true);
//        } else if (photonVision.hasTargets(lastPipelineResult) && limelight.isTargetFound()) {
//            if (limelight.getY() > photonVision.getRobotPoseRelativeToAprilTag(lastPipelineResult).getY()) {
//                visionMode = Enums.VisionMode.PHOTON_VISION;
//            } else {
//                visionMode = Enums.VisionMode.LIME_LIGHT;
//            }
//        } else if (limelight.isTargetFound()) {
            visionMode = Enums.VisionMode.LIME_LIGHT;
//        } else if (photonVision.hasTargets(lastPipelineResult)) {
//            visionMode = Enums.VisionMode.PHOTON_VISION;
//        } else {
//            end(true);
//        }
    }

    @Override
    public void execute() {
//        if (!photonVision.hasTargets(lastPipelineResult) && !limelight.isTargetFound()) {
//            end(true);
//        }

        if (visionMode == Enums.VisionMode.PHOTON_VISION) {
            lastPipelineResult = photonVision.getLatestResult();
            final Pose2d targetPose = photonVision.getRobotPoseRelativeToAprilTag(lastPipelineResult);

            targetErrorY = targetPose.getY();
            targetErrorX = targetPose.getX();

            int tagID = photonVision.targetId(lastPipelineResult);

            if (tagID == 5 || tagID == 4) { //Offset for substation
                targetErrorY += 0.5;
            }

            swerve.drive(
//                    xPhotonPIDController.calculate(targetErrorX),
                    0,
                    yPhotonPIDController.calculate(targetErrorY),
                    0,
                    false
            );

        } else if (visionMode == Enums.VisionMode.LIME_LIGHT) {
            targetErrorX = limelight.getX() + LIMELIGHT_X_OFFSET;
            swerve.faceDirection(
                    0,
                    xLimelightPIDController.calculate(targetErrorX),
                    180,
                    true
            );


        }
    }

    @Override
    public boolean isFinished() {
        return MathMethods.withinBand(targetErrorX, .55) && timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
        timer.stop();
    }
}
