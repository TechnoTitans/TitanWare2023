package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;

public class SwerveAlignment extends CommandBase {
    private final Swerve swerve;
    private final SwerveDriveOdometry odometry;
    private final Limelight limelight;
    private final PhotonVision photonVision;
    private double targetY;
    private double targetError;
    private PIDController pidController;


    public SwerveAlignment(Swerve swerve, SwerveDriveOdometry odometry, Limelight limelight, PhotonVision photonVision) {
        this.swerve = swerve;
        this.odometry = odometry;
        this.limelight = limelight;
        this.photonVision = photonVision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        targetY = odometry.getPoseMeters().getY()+ photonVision.getRobotPoseRelativeToAprilTag().getY();
        pidController = new PIDController(0.6, 0, 0);
    }

    @Override
    public void execute() {
//        if (limelight.isTargetFound()){
//            swerve.faceDirection(limelight.getSwerveError(), 0, 0, true);
//        }
        if (photonVision.hasResults()) {
            SmartDashboard.putString("vision data", photonVision.getRobotTransformationRelativeToAprilTag().toString());
//            Translation2d errorPose2d = targetTranslation2d.minus(odometry.getPoseMeters().getTranslation());
        }
        targetError = targetY - odometry.getPoseMeters().getY();
        swerve.faceDirection(0, pidController.calculate(targetError), 0, false);
    }

    @Override
    public boolean isFinished() {
//        return limelight.isTargetAligned();
        SmartDashboard.putNumber("err", targetError);
        return targetError < 0.2 && -0.2 < targetError;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
