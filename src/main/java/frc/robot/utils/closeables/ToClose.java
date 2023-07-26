package frc.robot.utils.closeables;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public final class ToClose {
    private static final List<AutoCloseable> toClose = new ArrayList<>();
    private static final Thread closingThread = new Thread(() -> {
        for (final AutoCloseable autoCloseable : toClose) {
            try {
                autoCloseable.close();
            } catch (final Exception exception) {
                DriverStation.reportError(
                        String.format("Failed to close resource: %s", exception), exception.getStackTrace()
                );
            }
        }
    });

    private static boolean hasHooked = false;

    private ToClose() {}

    public static void add(final AutoCloseable autoCloseable) {
        toClose.add(autoCloseable);
    }

    public static void hook() {
        if (hasHooked) {
            throw new RuntimeException("hook() can only be invoked once!");
        }

        hasHooked = true;
        Runtime.getRuntime().addShutdownHook(closingThread);
    }
}
