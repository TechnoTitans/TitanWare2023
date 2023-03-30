package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;

public class AutoAlignment2 extends CommandBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final XboxController mainController;
    private final Profiler driverProfile;
    private final PIDController xLimelightPIDController;
    private Pose2d targetPose;

    public AutoAlignment2(Swerve swerve, SwerveDrivePoseEstimator poseEstimator, XboxController mainController) {
        this.swerve = swerve;
        this.mainController = mainController;
        this.poseEstimator = poseEstimator;
        this.driverProfile = Profiler.getProfile();
        this.xLimelightPIDController = new PIDController(0.1, 0, 0);

        addRequirements(swerve);
    }

    public void setTarget(Enums.GridPositions state) {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        Pose2d LEFT = new Pose2d(),
                CENTER = new Pose2d(),
                RIGHT = new Pose2d();

        if (MathMethods.poseWithinArea(currentPose, Constants.Grid.LEFTBOTTOM, Constants.Grid.LEFTTOP)) {
            LEFT = Constants.Grid.LEFT.LEFT;
            CENTER = Constants.Grid.LEFT.CUBE;
            RIGHT = Constants.Grid.LEFT.RIGHT;
        } else if (MathMethods.poseWithinArea(currentPose, Constants.Grid.CENTERBOTTOM, Constants.Grid.CENTERTOP)) {
            LEFT = Constants.Grid.CENTER.LEFT;
            CENTER = Constants.Grid.CENTER.CUBE;
            RIGHT = Constants.Grid.CENTER.RIGHT;
        } else if (MathMethods.poseWithinArea(currentPose, Constants.Grid.RIGHTBOTTOM, Constants.Grid.RIGHTTOP)) {
            LEFT = Constants.Grid.RIGHT.LEFT;
            CENTER = Constants.Grid.RIGHT.CUBE;
            RIGHT = Constants.Grid.RIGHT.RIGHT;
        }
        switch (state) {
            case LEFT:
                targetPose = LEFT;
                break;
            case CENTER:
                targetPose = CENTER;
                break;
            case RIGHT:
                targetPose = RIGHT;
                break;
            default:
                break;
        }
        this.schedule();
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Transform2d poseError = poseEstimator.getEstimatedPosition().minus(targetPose);
        double frontBack = MathMethods.deadband(mainController.getLeftY(), 0.1) *
                Constants.Swerve.TELEOP_MAX_SPEED *
                driverProfile.getThrottleSensitivity();
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
