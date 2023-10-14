package frc.robot.utils.safety;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class SubsystemEStop {
    public static class StopBehavior {
        public static final StopBehavior DS_E_STOP = new StopBehavior(Kind.INSTANT);

        public enum Kind {
            INSTANT,
            AFTER_DURATION
        }

        private final Kind kind;
        private final double timeSeconds;
        public StopBehavior(final Kind kind, final double timeSeconds) {
            this.kind = kind;
            this.timeSeconds = timeSeconds;
        }

        public StopBehavior(final Kind kind) {
            this(kind, 0);
        }

        public Kind getKind() {
            return kind;
        }

        public double getTimeSeconds() {
            return timeSeconds;
        }

        // TODO: maybe include a message in StopBehavior?
        @Override
        public String toString() {
            return String.format("StopBehavior(%s, %.2f)", kind, timeSeconds);
        }
    }

    public static class StopConditionProvider<T> {
        private final Function<T, Boolean> shouldStop;
        private final Supplier<T> stopParamSupplier;
        private final StopBehavior stopBehavior;

        private final StopBehavior.Kind kind;
        private final double timeSeconds;

        private final Timer timer;

        public StopConditionProvider(
                final Function<T, Boolean> shouldStop,
                final Supplier<T> stopParamSupplier,
                final StopBehavior stopBehavior
        ) {
            this.shouldStop = shouldStop;
            this.stopParamSupplier = stopParamSupplier;
            this.stopBehavior = stopBehavior;

            this.kind = stopBehavior.getKind();
            this.timeSeconds = stopBehavior.getTimeSeconds();

            this.timer = new Timer();
            timer.start();
        }

        public static boolean isDisabledState() {
            return RobotState.isDisabled() || RobotState.isEStopped();
        }

        public boolean shouldStop() {
            return switch (kind) {
                case INSTANT -> shouldStop.apply(stopParamSupplier.get());
                case AFTER_DURATION -> {
                    final T stopParam = stopParamSupplier.get();
                    if (!shouldStop.apply(stopParam) || isDisabledState()) {
                        timer.reset();
                        yield timeSeconds <= 0;
                    }

                    yield timer.hasElapsed(timeSeconds);
                }
            };
        }

        public void restart() {
            timer.restart();
        }

        public StopBehavior getStopBehavior() {
            return stopBehavior;
        }

        public static StopConditionProvider<Double> instantDoubleLimit(
                final Supplier<Double> measurementSupplier,
                final double lowerLimit,
                final double upperLimit
        ) {
            return new StopConditionProvider<>(
                    (measurement) -> measurement < lowerLimit || measurement > upperLimit,
                    measurementSupplier,
                    new StopBehavior(SubsystemEStop.StopBehavior.Kind.INSTANT)
            );
        }
    }

    private final List<StopConditionProvider<?>> stopConditionProviders;
    private final Consumer<StopBehavior> onEStop;
    private final Supplier<Boolean> override;

    private boolean eStopped = false;

    public SubsystemEStop(
            final List<StopConditionProvider<?>> stopConditionProviders,
            final Consumer<StopBehavior> onEStop,
            final Supplier<Boolean> override
    ) {
        this.stopConditionProviders = stopConditionProviders;
        this.onEStop = onEStop;
        this.override = override;
    }

    public void eStop(final StopBehavior stopBehavior) {
        if (eStopped || override.get()) {
            return;
        }

        eStopped = true;
        onEStop.accept(stopBehavior);
    }

    public void clearEStop() {
        if (eStopped) {
            eStopped = false;
            for (final StopConditionProvider<?> stopConditionProvider : stopConditionProviders) {
                stopConditionProvider.restart();
            }
        }
    }

    public boolean isEStopped() {
        return eStopped;
    }

    public void periodic() {
        if (RobotState.isEStopped() && !isEStopped()) {
            eStop(StopBehavior.DS_E_STOP);
        }

        for (final StopConditionProvider<?> stopConditionProvider : stopConditionProviders) {
            if (stopConditionProvider.shouldStop() && !isEStopped()) {
                eStop(stopConditionProvider.getStopBehavior());
            }
        }

        if (isEStopped() && override.get()) {
            clearEStop();
        }
    }
}
