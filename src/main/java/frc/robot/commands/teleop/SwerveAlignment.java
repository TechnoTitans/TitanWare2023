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
    private final PIDController xPidController, yPidController;
    private Enums.VisionMode visionMode;

    public SwerveAlignment(Swerve swerve, Limelight limelight, PhotonVision photonVision) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.photonVision = photonVision;
        this.xPidController = new PIDController(2, 0.1, 0);
        this.yPidController = new PIDController(3, 0.1, 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (!photonVision.hasTargets() && !limelight.isTargetFound()) {
            end(true);
        } else if (photonVision.hasTargets() && limelight.isTargetFound()) {
            if (limelight.getY() > photonVision.getRobotPoseRelativeToAprilTag().getY()) {
                visionMode = Enums.VisionMode.PhotonVision;
            } else {
                visionMode = Enums.VisionMode.LimeLight;
            }
        } else if (limelight.isTargetFound()) {
            visionMode = Enums.VisionMode.LimeLight;
        } else if (photonVision.hasTargets()) {
            visionMode = Enums.VisionMode.PhotonVision;
        } else {
            end(true);
        }
    }

    private final double xOffset = -0.46;

    @Override
    public void execute() {
        if (visionMode == Enums.VisionMode.PhotonVision && photonVision.hasTargets()) {
            Pose2d targetPose = photonVision.getRobotPoseRelativeToAprilTag();
            targetErrorY = targetPose.getY();
            targetErrorX = targetPose.getX() + xOffset;
        } else if (visionMode == Enums.VisionMode.LimeLight && limelight.isTargetFound()) {
            targetErrorY = limelight.getY();
            targetErrorX = limelight.getX();
        }
        SmartDashboard.putNumber("x", targetErrorX);
        swerve.faceDirection(xPidController.calculate(targetErrorX), yPidController.calculate(targetErrorY), 0, false);
    }

    @Override
    public boolean isFinished() {
        return (MathMethods.withinBand(targetErrorY, 0.05) && MathMethods.withinBand(targetErrorX, 0.1));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
