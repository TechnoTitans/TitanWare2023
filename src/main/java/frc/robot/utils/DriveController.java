package frc.robot.utils;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Custom version of a @HolonomicDriveController specifically for following PathPlanner paths
 *
 * <p>This controller adds the following functionality over the WPILib version: - calculate() method
 * takes in a PathPlannerState directly - Continuous input is automatically enabled for the rotation
 * controller - Holonomic angular velocity is used as a feedforward for the rotation controller,
 * which no longer needs to be a @ProfiledPIDController
 */

@SuppressWarnings("unused")
public class DriveController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private Translation2d translationError = new Translation2d();
    private Rotation2d rotationError = new Rotation2d();
    private Pose2d tolerance = new Pose2d();

    /**
     * Constructs a PPHolonomicDriveController
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
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;

        // Auto-configure continuous input for rotation controller
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final Translation2d translationTolerance = this.tolerance.getTranslation();
        final Rotation2d rotationTolerance = this.tolerance.getRotation();

        return Math.abs(this.translationError.getX()) < translationTolerance.getX()
                && Math.abs(this.translationError.getY()) < translationTolerance.getY()
                && Math.abs(this.rotationError.getRadians()) < rotationTolerance.getRadians();
    }

    /**
     * Sets the pose error whic is considered tolerance for use with atReference()
     *
     * @param tolerance The pose error which is tolerable
     */
    public void setTolerance(final Pose2d tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Calculates the next output of the holonomic drive controller
     *
     * @param currentPose    The current pose
     * @param referenceState The desired trajectory state
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(
            final Pose2d currentPose,
            final PathPlannerState referenceState
    ) {
        this.translationError = referenceState.poseMeters.relativeTo(currentPose).getTranslation();
        this.rotationError = referenceState.holonomicRotation.minus(currentPose.getRotation());

        double xFeedback = this.xController.calculate(currentPose.getX(), referenceState.poseMeters.getX());
        double yFeedback = this.yController.calculate(currentPose.getY(), referenceState.poseMeters.getY());
        double rotationFeedback = this.rotationController.calculate(
                currentPose.getRotation().getRadians(), referenceState.holonomicRotation.getRadians());

        return new ChassisSpeeds(xFeedback, yFeedback, rotationFeedback);
    }
}
