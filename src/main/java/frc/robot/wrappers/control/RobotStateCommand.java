package frc.robot.wrappers.control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotStateCommand extends InstantCommand {
    public RobotStateCommand(final Runnable runnable, final RobotState state) {
        super(() -> {
            if (
                    (state == RobotState.DISABLED && DriverStation.isDisabled())
                            || (state == RobotState.TELEOP && DriverStation.isTeleopEnabled())
                            || (state == RobotState.AUTONOMOUS && DriverStation.isAutonomousEnabled())
                            || (state == RobotState.TEST && DriverStation.isTestEnabled())
            ) {
                runnable.run();
            }
        });
    }

    public RobotStateCommand(final Runnable runnable) {
        this(runnable, RobotState.TELEOP);
    }

    public enum RobotState {
        DISABLED,
        TELEOP,
        AUTONOMOUS,
        TEST
    }
}
