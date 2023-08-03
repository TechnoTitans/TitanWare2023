package frc.robot.utils.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControllerUtils {
    public final static double STICK_TO_GYRO_ROTATION = -1;
    public final static Rotation2d STICK_TO_FIELD_RELATIVE_ROTATION = Rotation2d.fromDegrees(-90);

    // TODO: document
    public static Translation2d getStickXYSquaredInput(
            final double xInput,
            final double yInput,
            final double deadband,
            final double scalar,
            final double sensitivity,
            final double weight
    ) {
        final double magnitude = Math.hypot(xInput, yInput);
        final Rotation2d direction = new Rotation2d(xInput, yInput);

        final double deadbandMagnitude = applyDeadband(magnitude, deadband);
        final double squaredMagnitude = Math.copySign(deadbandMagnitude * deadbandMagnitude, deadbandMagnitude);

        final double weightedMagnitude = applyWeights(squaredMagnitude, scalar, sensitivity, weight);
        return new Pose2d(new Translation2d(), direction)
                .transformBy(new Transform2d(new Translation2d(weightedMagnitude, 0), new Rotation2d()))
                .getTranslation();
    }

    public static double getStickSquaredInput(
            final double input,
            final double deadband,
            final double scalar,
            final double sensitivity,
            final double weight
    ) {
        return applyWeights(applyDeadband(Math.copySign(input * input, input), deadband), scalar, sensitivity, weight);
    }

    public static double applyDeadband(final double input, final double deadband) {
        // yes, this negative sign does need to exist because controller stick up is -1 not 1
        return -MathUtil.applyDeadband(input, Math.abs(deadband));
    }

    public static double applyWeights(
            final double input,
            final double scalar,
            final double sensitivity,
            final double weight
    ) {
        return input * scalar * sensitivity * weight;
    }

    public static Rotation2d getFieldRelativeAngleFromStickInputs(final double xInput, final double yInput) {
        return Rotation2d.fromRadians(Math.atan2(yInput, xInput))
                .rotateBy(STICK_TO_FIELD_RELATIVE_ROTATION)
                .times(STICK_TO_GYRO_ROTATION);
    }

    public static Command getRumbleForDurationCommand(
            final GenericHID controller,
            final GenericHID.RumbleType rumbleType,
            final double value,
            final double timeSeconds
    ) {
        return Commands.sequence(
                Commands.runOnce(() -> controller.setRumble(rumbleType, value)),
                Commands.waitSeconds(timeSeconds),
                Commands.runOnce(() -> controller.setRumble(rumbleType, 0))
        );
    }
}
