package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.sensors.vision.Limelight;

public class SwerveAlignment extends CommandBase {
    private final Swerve swerve;
    private final Limelight limelight;
    private final XboxController mainController;
    private final Timer timer;
    private final Profiler driverProfile;
    private final PIDController xLimelightPIDController;

    public SwerveAlignment(Swerve swerve, Limelight limelight, XboxController mainController) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.mainController = mainController;
        this.timer = new Timer();
        this.driverProfile = Profiler.getProfile();
        this.xLimelightPIDController = new PIDController(0.1, 0, 0);

        addRequirements(swerve);
    }

    public void setTrackMode(Enums.LimelightPipelines pipeline) {
        switch (pipeline) {
            case LEFT:
                limelight.changePipeline(0d);
                break;
            case RIGHT:
                limelight.changePipeline(1d);
                break;
        }
        this.schedule();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double frontBack = MathMethods.deadband(mainController.getLeftY(), 0.1) * Constants.Swerve.TELEOP_MAX_SPEED * driverProfile.getThrottleSensitivity();
        swerve.faceDirection(
                -frontBack * driverProfile.getThrottleSlowWeight(),
                xLimelightPIDController.calculate(limelight.getX()),
                180,
                true
        );
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 3;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.stop();
    }
}
