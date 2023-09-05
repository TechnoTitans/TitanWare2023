package frc.robot.utils.auto;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Custom version of a {@link edu.wpi.first.math.controller.HolonomicDriveController} specifically for
 * following PathPlanner paths
 *
 * <p>
 *    This controller adds the following functionality over the WPILib version: - calculate() method
 *    takes in a {@link PathPlannerState} directly - Continuous input is automatically enabled for the rotation
 *    controller - Holonomic angular velocity is used as a feedforward for the rotation controller,
 *    which no longer needs to be a {@link edu.wpi.first.math.controller.ProfiledPIDController}
 * </p>
 */

public class DriveController {
    /**
     * Determines if the DriveController will use x-axis FeedForward obtained from PathPlanner in addition
     * to PID control
     */
    private final boolean useXFF;
    /**
     * Determines if the DriveController will use y-axis FeedForward obtained from PathPlanner in addition
     * to PID control
     */
    private final boolean useYFF;
    /**
     * Determines if the DriveController will use rotational FeedForward obtained from PathPlanner in addition
     * to PID control
     */
    private final boolean useRotFF;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private final boolean isEnabled;

    /**
     * Constructs a DriveController
     *
     * @param xController        A PID controller to respond to error in the field-relative X direction
     * @param yController        A PID controller to respond to error in the field-relative Y direction
     * @param rotationController A PID controller to respond to error in rotation
     * @param isEnabled          Use PID control
     * @param useXFF             Use x-axis FeedForward obtained from PathPlanner
     * @param useYFF             Use y-axis FeedForward obtained from PathPlanner
     * @param useRotFF           Use rotational FeedForward obtained from PathPlanner
     */
    public DriveController(
            final PIDController xController,
            final PIDController yController,
            final PIDController rotationController,
            final boolean isEnabled,
            final boolean useXFF,
            final boolean useYFF,
            final boolean useRotFF
    ) {
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;
        this.isEnabled = isEnabled;
        this.useXFF = useXFF;
        this.useYFF = useYFF;
        this.useRotFF = useRotFF;

        // Auto-configure continuous input for rotation controller
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Constructs a DriveController (without any FeedForward input)
     *
     * @param xController        A PID controller to respond to error in the field-relative X direction
     * @param yController        A PID controller to respond to error in the field-relative Y direction
     * @param rotationController A PID controller to respond to error in rotation
     * @param isEnabled          Use PID control
     * @param useFF              Use FeedForward
     */
    public DriveController(
            final PIDController xController,
            final PIDController yController,
            final PIDController rotationController,
            final boolean isEnabled,
            final boolean useFF
    ) {
        this(xController, yController, rotationController, isEnabled, useFF, useFF, useFF);
    }

    /**
     * Constructs a DriveController (without any FeedForward input)
     *
     * @param xController        A PID controller to respond to error in the field-relative X direction
     * @param yController        A PID controller to respond to error in the field-relative Y direction
     * @param rotationController A PID controller to respond to error in rotation
     */
    public DriveController(
            final PIDController xController,
            final PIDController yController,
            final PIDController rotationController
    ) {
        this(xController, yController, rotationController, true, true);
    }

    /**
     * Resets the state of the DriveController by resetting accumulated error on all PID controllers
     * @see PIDController#reset()
     */
    public void reset() {
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    /**
     * Calculates the next output of the holonomic drive controller
     *
     * @param currentPose    The current pose
     * @param wantedState The desired trajectory state
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(final Pose2d currentPose, final PathPlannerState wantedState) {
        final double xFF = useXFF
                ? wantedState.velocityMetersPerSecond * wantedState.poseMeters.getRotation().getCos()
                : 0;
        final double yFF = useYFF
                ? wantedState.velocityMetersPerSecond * wantedState.poseMeters.getRotation().getSin()
                : 0;
        final double rotationFF = useRotFF
                ? wantedState.holonomicAngularVelocityRadPerSec
                : 0;

        if (!this.isEnabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, rotationFF, currentPose.getRotation());
        }

        double xFeedback =
                this.xController.calculate(currentPose.getX(), wantedState.poseMeters.getX());
        double yFeedback =
                this.yController.calculate(currentPose.getY(), wantedState.poseMeters.getY());
        double rotationFeedback =
                this.rotationController.calculate(
                        currentPose.getRotation().getRadians(), wantedState.holonomicRotation.getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFF + rotationFeedback,
                currentPose.getRotation()
        );
    }
}
