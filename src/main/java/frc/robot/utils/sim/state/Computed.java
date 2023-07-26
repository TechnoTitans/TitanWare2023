package frc.robot.utils.sim.state;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class Computed<T> implements State<T> {
    public static final long SEC_TO_MICROSECOND = 1_000_000;
    public static final long LOOP_PERIOD_MICROSECONDS = (long) Constants.LOOP_PERIOD_SECONDS * SEC_TO_MICROSECOND;

    private final Supplier<T> tSupplier;
    private final List<State<?>> dependents;
    private final boolean hasDependents;
    private final State<?> singularDependent;
    private final boolean onlyOneDependent;
    private long lastUpdateFPGA;
    private T lastT;

    public Computed(final Supplier<T> tSupplier, final State<?>... dependents) {
        this.tSupplier = tSupplier;
        this.dependents = Arrays.stream(dependents).toList();

        this.hasDependents = !this.dependents.isEmpty();
        this.onlyOneDependent = this.dependents.size() == 1;
        this.singularDependent = this.onlyOneDependent ? this.dependents.get(0) : null;

        rawUpdate();
    }

    @Override
    public void set(final T t) {
        lastUpdateFPGA = Logger.getInstance().getTimestamp();
        lastT = t;
    }

    private void rawUpdate() {
        set(tSupplier.get());
    }

    @Override
    public boolean update() {
        final long currentFPGATime = Logger.getInstance().getTimestamp();
        if (Math.abs(currentFPGATime - lastUpdateFPGA) < LOOP_PERIOD_MICROSECONDS) {
            return false;
        }

        if (!hasDependents) {
            return false;
        }

        if (onlyOneDependent) {
            final boolean didUpdate = singularDependent.update();
            if (didUpdate) {
                lastT = tSupplier.get();
            }
        } else {
            boolean didAtLeastOneUpdate = false;
            for (final State<?> dependent : dependents) {
                if (!didAtLeastOneUpdate) {
                    didAtLeastOneUpdate = dependent.update();
                }
            }

            if (didAtLeastOneUpdate) {
                lastT = tSupplier.get();
            }
        }

        lastUpdateFPGA = currentFPGATime;
        return true;
    }

    @Override
    public T get() {
        update();
        return lastT;
    }
}
