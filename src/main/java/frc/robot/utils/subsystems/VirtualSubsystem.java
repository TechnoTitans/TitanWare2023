package frc.robot.utils.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.utils.closeables.ToClose;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class VirtualSubsystem implements Subsystem {
    private static final Set<VirtualSubsystem> registeredSubsystems = new HashSet<>();
    private static final HashMap<VirtualSubsystem, Notifier> subsystemNotifierMap = new HashMap<>();

    public VirtualSubsystem(final double loopPeriodSeconds) {
        if (loopPeriodSeconds != Constants.LOOP_PERIOD_SECONDS) {
            final Notifier subsystemNotifier = new Notifier(this::periodic);
            ToClose.add(subsystemNotifier);

            subsystemNotifier.startPeriodic(loopPeriodSeconds);
            subsystemNotifierMap.put(this, subsystemNotifier);
        } else {
            CommandScheduler.getInstance().registerSubsystem(this);
        }

        registeredSubsystems.add(this);
    }

    public VirtualSubsystem() {
        this(Constants.LOOP_PERIOD_SECONDS);
    }

    /**
     * Get the registered {@link VirtualSubsystem}s, this returns an internal
     * mutable reference {@link Set}, so it should not be modified.
     * @return the {@link Set} of registered {@link VirtualSubsystem}s
     */
    @SuppressWarnings("unused")
    public static Set<VirtualSubsystem> getRegisteredSubsystems() {
        return registeredSubsystems;
    }

    /**
     * Get the {@link Notifier} of a registered {@link VirtualSubsystem} that has a
     * non-standard loop period != {@link Constants#LOOP_PERIOD_SECONDS}
     * @param subsystem the {@link VirtualSubsystem} to get the {@link Notifier} for
     * @return the {@link Notifier} associated with the {@link VirtualSubsystem}
     */
    @SuppressWarnings("unused")
    public static Notifier getNonLoopPeriodNotifier(final VirtualSubsystem subsystem) {
        final Notifier notifier = subsystemNotifierMap.get(subsystem);
        if (notifier == null) {
            throw new RuntimeException(
                    "Attempted to get Notifier for non-registered or non-notifier VirtualSubsystem!"
            );
        }

        return notifier;
    }
}
