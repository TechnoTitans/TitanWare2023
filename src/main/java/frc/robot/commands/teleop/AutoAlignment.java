package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathUtils;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.TitanBoard;

public class AutoAlignment extends CommandBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final XboxController mainController;
    private final PIDController alignPIDController;
    private Pose2d targetPose;

    private String gridSectionName = "None";

    public AutoAlignment(
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final XboxController mainController
    ) {
        this.swerve = swerve;
        this.mainController = mainController;
        this.poseEstimator = poseEstimator;
        this.alignPIDController = new PIDController(0.7, 0, 0);

        TitanBoard.addString("gridSection", () -> gridSectionName);

        addRequirements(swerve);
    }

    public void setState(final Enums.GridPositions state) {
        final Pose2d currentPose = poseEstimator.getEstimatedPosition();
        final Pose2d LEFT, CENTER, RIGHT;

        if (PoseUtils.poseWithinArea(currentPose, Constants.Field.LEFT_BOTTOM, Constants.Field.LEFT_TOP)) { //LEFT SIDE OF GRID
            gridSectionName = "LEFT";
            LEFT = Constants.Field.LEFT.LEFT;
            CENTER = Constants.Field.LEFT.CUBE;
            RIGHT = Constants.Field.LEFT.RIGHT;
        } else if (PoseUtils.poseWithinArea(currentPose, Constants.Field.CENTER_BOTTOM, Constants.Field.CENTER_TOP)) { // CENTER OF GRID
            gridSectionName = "CENTER";
            LEFT = Constants.Field.CENTER.LEFT;
            CENTER = Constants.Field.CENTER.CUBE;
            RIGHT = Constants.Field.CENTER.RIGHT;
        } else if (PoseUtils.poseWithinArea(currentPose, Constants.Field.RIGHT_BOTTOM, Constants.Field.RIGHT_TOP)) { // RIGHT OF GRID
            gridSectionName = "RIGHT";
            LEFT = Constants.Field.RIGHT.LEFT;
            CENTER = Constants.Field.RIGHT.CUBE;
            RIGHT = Constants.Field.RIGHT.RIGHT;
        } else {
            return;
        }

        switch (state) {
            case LEFT ->
                    targetPose = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? LEFT : RIGHT; // LEFT CONE OF SELECTED GRID AREA
            case CENTER ->
                    targetPose = CENTER; // CENTER CUBE OF SELECTED GRID AREA
            case RIGHT ->
                    targetPose = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? RIGHT : LEFT; // LEFT CONE OF SELECTED GRID AREA
            default -> {
                return;
            }
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
        final Pose2d transformedPose = PoseUtils.transformRobotPose(poseEstimator.getEstimatedPosition());
        final Transform2d poseError = transformedPose.minus(targetPose);

        final double frontBack = MathUtils.deadband(mainController.getLeftY(), 0.01) *
                Constants.Swerve.TELEOP_MAX_SPEED *
                Profiler.getDriverProfile().getThrottleSensitivity();

        swerve.faceDirection(
                frontBack * Profiler.getSwerveSpeed().getThrottleWeight(),
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
