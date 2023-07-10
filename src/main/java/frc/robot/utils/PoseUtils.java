package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class PoseUtils {
    public static final double EPSILON = 1E-9;

    public static boolean poseWithinArea(
            final Pose2d currentPose,
            final Translation2d point1,
            final Translation2d point2
    ) {
        final Pose2d transformedPose = localizeRobotPoseOnBlueAlliance(currentPose);
        final Translation2d transformedTranslation = transformedPose.getTranslation();

        return transformedTranslation.getX() > point1.getX()
                && transformedTranslation.getX() < point2.getX()
                && transformedTranslation.getY() > point1.getY()
                && transformedTranslation.getY() < point2.getY();
    }

    public static Pose2d flipAlliancePose(final Pose2d poseToFlip) {
        return poseToFlip.relativeTo(Constants.Field.FLIPPING_POSE);
    }

    public static Pose2d transformRobotPose(final Pose2d poseToFlip) {
        return DriverStation.getAlliance() == DriverStation.Alliance.Red
                ? flipAlliancePose(poseToFlip)
                : poseToFlip;
    }

    public static Pose2d localizeRobotPoseOnBlueAlliance(final Pose2d originalPose) {
        return DriverStation.getAlliance() == DriverStation.Alliance.Red
                        ? new Pose2d(
                            originalPose.getX(),
                            Constants.Field.FIELD_WIDTH_Y_METERS - originalPose.getY(),
                            originalPose.getRotation().times(-1))
                        : originalPose;
    }

    public static Pose2d transformGridPose(final Pose2d originalPose) {
        return DriverStation.getAlliance() == DriverStation.Alliance.Red
                ? new Pose2d(
                    Constants.Field.FIELD_LENGTH_X_METERS - originalPose.getX(),
                    originalPose.getY(),
                    originalPose.getRotation().times(-1))
                : originalPose;
    }

    public static Twist2d log(final Pose2d transform) {
        final double dTheta = transform.getRotation().getRadians();
        final double halfDTheta = 0.5 * dTheta;
        final double cosMinusOne = transform.getRotation().getCos() - 1.0;

        final double halfThetaByTanOfHalfTheta;
        if (Math.abs(cosMinusOne) < EPSILON) {
            halfThetaByTanOfHalfTheta = 1.0 - 1.0 / 12.0 * dTheta * dTheta;
        } else {
            halfThetaByTanOfHalfTheta = -(halfDTheta * transform.getRotation().getSin()) / cosMinusOne;
        }

        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halfThetaByTanOfHalfTheta, -halfDTheta));

        return new Twist2d(translation_part.getX(), translation_part.getY(), dTheta);
    }
}
