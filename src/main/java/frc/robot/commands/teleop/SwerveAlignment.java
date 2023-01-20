package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class SwerveAlignment extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;
    private final PhotonVision photonVision;
    private double targetErrorX, targetErrorY;
    private PIDController xPidController, yPidController;

    public SwerveAlignment(Swerve swerve, Limelight limelight, PhotonVision photonVision) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.photonVision = photonVision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        xPidController = new PIDController(0.2, 0.1, 0);
        yPidController = new PIDController(3, 0.1, 0);
    }

    @Override
    public void execute() {
//        if (limelight.isTargetFound()){
//            swerve.faceDirection(limelight.getSwerveError(), 0, 0, true);
//        }
        targetErrorY = photonVision.getRobotPoseRelativeToAprilTag().getY();
        targetErrorX = photonVision.getRobotPoseRelativeToAprilTag().getX();
        SmartDashboard.putNumber("x", targetErrorX);
        swerve.faceDirection(xPidController.calculate(targetErrorX), yPidController.calculate(targetErrorY), 0, false);
    }

    @Override
    public boolean isFinished() {
//        return limelight.isTargetAligned();
        return (MathMethods.withinBand(targetErrorY, 0.05) && MathMethods.withinBand(targetErrorX, 0.1));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
