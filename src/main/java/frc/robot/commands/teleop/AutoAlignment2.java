package frc.robot.commands.teleop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.sensors.vision.Limelight;

public class AutoAlignment2 extends CommandBase {
    private final Swerve swerve;
    private final XboxController mainController;
    private final Profiler driverProfile;
    private final PIDController xLimelightPIDController;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d targetPose;

    public AutoAlignment2(Swerve swerve, XboxController mainController, SwerveDrivePoseEstimator poseEstimator) {
        this.swerve = swerve;
        this.mainController = mainController;
        this.poseEstimator = poseEstimator;
        this.driverProfile = Profiler.getProfile();
        this.xLimelightPIDController = new PIDController(0.1, 0, 0);

        addRequirements(swerve);
    }

    public void setTarget(Enums.targets state) {
        switch (state) {
            case one:
                targetPose = new Pose2d(1,0,Rotation2d.fromDegrees(180));
                break;
            case two:
                targetPose = new Pose2d(2,0,Rotation2d.fromDegrees(180));
                break;
            default:
                break;
        }
        this.targetPose = targetPose;
        this.schedule();
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Transform2d poseError = poseEstimator.getEstimatedPosition().minus(
                targetPose);
        double frontBack = MathMethods.deadband(mainController.getLeftY(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();
        swerve.faceDirection(
                frontBack * driverProfile.getThrottleNormalWeight(),
                xLimelightPIDController.calculate(poseError.getX()),
                180,
                true
        );
    }

    @Override
    public boolean isFinished() {
        return !mainController.getRightBumper() && !mainController.getLeftBumper();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
