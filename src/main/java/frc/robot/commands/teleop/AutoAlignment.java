package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.DriveController;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.utils.PoseUtils;

public class AutoAlignment extends CommandBase {
    private final Swerve swerve;
    private final Field2d field2d;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final XboxController mainController;
    private final Profiler driverProfile;
    private final PIDController alignPIDController;
    private Pose2d targetPose;

    public AutoAlignment(Swerve swerve, SwerveDrivePoseEstimator poseEstimator, XboxController mainController, Field2d field2d) {
        this.swerve = swerve;
        this.field2d = field2d;
        this.mainController = mainController;
        this.poseEstimator = poseEstimator;
        this.driverProfile = Profiler.getProfile();
        this.alignPIDController = new PIDController(0.6, 0, 0);

        addRequirements(swerve);
    }

    public void setState(Enums.GridPositions state) {
        final Pose2d currentPose = poseEstimator.getEstimatedPosition();
        final Pose2d LEFT, CENTER, RIGHT;

        if (PoseUtils.poseWithinArea(currentPose, Constants.Grid.LEFTBOTTOM, Constants.Grid.LEFTTOP, field2d)) { //LEFT SIDE OF GRID
            LEFT = Constants.Grid.LEFT.LEFT;
            CENTER = Constants.Grid.LEFT.CUBE;
            RIGHT = Constants.Grid.LEFT.RIGHT;
        } else if (PoseUtils.poseWithinArea(currentPose, Constants.Grid.CENTERBOTTOM, Constants.Grid.CENTERTOP, field2d)) { // CENTER OF GRID
            LEFT = Constants.Grid.CENTER.LEFT;
            CENTER = Constants.Grid.CENTER.CUBE;
            RIGHT = Constants.Grid.CENTER.RIGHT;
        } else if (PoseUtils.poseWithinArea(currentPose, Constants.Grid.RIGHTBOTTOM, Constants.Grid.RIGHTTOP, field2d)) { // RIGHT OF GRID
            LEFT = Constants.Grid.RIGHT.LEFT;
            CENTER = Constants.Grid.RIGHT.CUBE;
            RIGHT = Constants.Grid.RIGHT.RIGHT;
        } else {
            return;
        }

        switch (state) {
            case LEFT:
                targetPose = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? LEFT : RIGHT; // LEFT CONE OF SELECTED GRID AREA
                break;
            case CENTER:
                targetPose = CENTER; // CENTER CUBE OF SELECTED GRID AREA
                break;
            case RIGHT:
                targetPose = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? RIGHT : LEFT; // LEFT CONE OF SELECTED GRID AREA
                break;
            default:
                return;
        }

        targetPose = PoseUtils.transformGridPose(targetPose);
        this.schedule();
    }

    @Override
    public void initialize() {
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Pose2d transformedPose = PoseUtils.transformRobotPose(poseEstimator.getEstimatedPosition());
        Transform2d poseError = transformedPose.minus(targetPose);

        double frontBack = MathMethods.deadband(mainController.getLeftY(), 0.1) *
                Constants.Swerve.TELEOP_MAX_SPEED *
                driverProfile.getThrottleSensitivity();

        swerve.faceDirection(
                frontBack * driverProfile.getThrottleWeight(),
                ((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? -1 : 1) * alignPIDController.calculate(poseError.getY()),
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
