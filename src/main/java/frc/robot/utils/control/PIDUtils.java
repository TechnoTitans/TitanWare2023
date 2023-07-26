package frc.robot.utils.control;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.utils.ctre.Phoenix6Utils;

public final class PIDUtils {
    private PIDUtils() {}

    public static void resetProfiledPIDControllerWithStatusSignal(
            final ProfiledPIDController profiledPIDController,
            final StatusSignal<Double> positionSignal,
            final StatusSignal<Double> velocitySignal
    ) {
        profiledPIDController.reset(Phoenix6Utils.latencyCompensateIfSignalIsGood(
                positionSignal,
                velocitySignal
        ));
    }
}
