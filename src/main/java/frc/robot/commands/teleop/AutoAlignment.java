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

public class AutoAlignment extends CommandBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final XboxController mainController;
    private final Profiler driverProfile;
    private final PIDController alignPIDController;
    private Pose2d targetPose;

    public AutoAlignment(Swerve swerve, SwerveDrivePoseEstimator poseEstimator, XboxController mainController) {
        this.swerve = swerve;
        this.mainController = mainController;
        this.poseEstimator = poseEstimator;
        this.driverProfile = Profiler.getProfile();
        this.alignPIDController = new PIDController(0.6, 0, 0);

        addRequirements(swerve);
    }

    public void setState(Enums.GridPositions state) {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        Pose2d LEFT, CENTER, RIGHT;

        if (MathMethods.poseWithinArea(currentPose, Constants.Grid.LEFTBOTTOM, Constants.Grid.LEFTTOP)) { //LEFT SIDE OF GRID
            LEFT = Constants.Grid.LEFT.LEFT;
            CENTER = Constants.Grid.LEFT.CUBE;
            RIGHT = Constants.Grid.LEFT.RIGHT;
        } else if (MathMethods.poseWithinArea(currentPose, Constants.Grid.CENTERBOTTOM, Constants.Grid.CENTERTOP)) { // CENTER OF GRID
            LEFT = Constants.Grid.CENTER.LEFT;
            CENTER = Constants.Grid.CENTER.CUBE;
            RIGHT = Constants.Grid.CENTER.RIGHT;
        } else if (MathMethods.poseWithinArea(currentPose, Constants.Grid.RIGHTBOTTOM, Constants.Grid.RIGHTTOP)) { // RIGHT OF GRID
            LEFT = Constants.Grid.RIGHT.LEFT;
            CENTER = Constants.Grid.RIGHT.CUBE;
            RIGHT = Constants.Grid.RIGHT.RIGHT;
        } else {
            return;
        }

        switch (state) {
            case LEFT:
                targetPose = LEFT; // LEFT CONE OF SELECTED GRID AREA
                break;
            case CENTER:
                targetPose = CENTER; // CENTER CUBE OF SELECTED GRID AREA
                break;
            case RIGHT:
                targetPose = RIGHT; // RIGHT CONE OF SELECTED GRID AREA
                break;
            default:
                return;
        }

        targetPose = MathMethods.transformPose(targetPose);

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
                frontBack * driverProfile.getThrottleWeight(),
                alignPIDController.calculate(poseError.getY()),
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
