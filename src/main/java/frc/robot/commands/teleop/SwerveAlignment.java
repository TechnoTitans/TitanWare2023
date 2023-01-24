package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class SwerveAlignment extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;
    private final PhotonVision photonVision;
    private double targetErrorX, targetErrorY;

    private final PIDController xPhotonPIDController = new PIDController(2, 0.1, 0);
    private final PIDController yPhotonPIDController = new PIDController(3, 0.1, 0);
    private final PIDController xLimelightPIDController = new PIDController(0.075, 0.1, 0);
    private final PIDController yLimelightPIDController = new PIDController(0.075, 0.1, 0);

    private final double PHOTON_X_OFFSET = -0.46;
    private final double LIMELIGHT_X_OFFSET = -0.5;

    private Enums.VisionMode visionMode;

    public SwerveAlignment(Swerve swerve, Limelight limelight, PhotonVision photonVision) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.photonVision = photonVision;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("LL state", "limelight ON called");
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_ON);
//        if (!photonVision.hasTargets() && !limelight.isTargetFound()) {
//            end(true);
//        } else

        if (photonVision.hasTargets() && limelight.isTargetFound()) {
            if (limelight.getY() > photonVision.getRobotPoseRelativeToAprilTag().getY()) {
                visionMode = Enums.VisionMode.PhotonVision;
            } else {
                visionMode = Enums.VisionMode.LimeLight;
            }
        } else if (limelight.isTargetFound()) {
            visionMode = Enums.VisionMode.LimeLight;
        } else if (photonVision.hasTargets()) {
            visionMode = Enums.VisionMode.PhotonVision;
        } //else {
//            end(true);
//        }
    }

    @Override
    public void execute() {
//        if (!photonVision.hasTargets() && !limelight.isTargetFound()) {
//            end(true);
//        }

        if (visionMode == Enums.VisionMode.PhotonVision && photonVision.hasTargets()) {
            final Pose2d targetPose = photonVision.getRobotPoseRelativeToAprilTag();

            targetErrorY = targetPose.getY();
            targetErrorX = targetPose.getX() + PHOTON_X_OFFSET;

            swerve.faceDirection(
                    xPhotonPIDController.calculate(targetErrorX),
                    yPhotonPIDController.calculate(targetErrorY),
                    0,
                    false
            );

        } else if (visionMode == Enums.VisionMode.LimeLight && limelight.isTargetFound()) {
            targetErrorY = limelight.calculateDistance();
            targetErrorX = limelight.getX();

            SmartDashboard.putNumber("LL X", targetErrorX);
            SmartDashboard.putNumber("LL Y", targetErrorY);

            swerve.faceDirection(
                    yLimelightPIDController.calculate(targetErrorY),
                    xLimelightPIDController.calculate(-targetErrorX),
                    0,
                    false
            );
        }
    }

    @Override
    public boolean isFinished() {
        return (MathMethods.withinBand(targetErrorY, 0.05) && MathMethods.withinBand(targetErrorX, 0.1));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        limelight.setLEDMode(Enums.LimeLightLEDState.LED_OFF);
        SmartDashboard.putString("LL state", "limelight OFF called");
    }
}
