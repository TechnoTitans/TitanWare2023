package frc.robot.utils.safety;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.utils.ctre.Phoenix6Utils;

import java.util.function.DoubleSupplier;

public class MeasurementObserver {
    private final DoubleSupplier measurementSupplier;
    private final DoubleSupplier measurementVelocitySupplier;
    private final DoubleSupplier setpointSupplier;

    public MeasurementObserver(
            final DoubleSupplier measurementSupplier,
            final DoubleSupplier measurementVelocitySupplier,
            final DoubleSupplier setpointSupplier
    ) {
        this.measurementSupplier = measurementSupplier;
        this.measurementVelocitySupplier = measurementVelocitySupplier;
        this.setpointSupplier = setpointSupplier;
    }

    public MeasurementObserver(
            final StatusSignal<Double> measurementSignal,
            final StatusSignal<Double> measurementVelocitySignal,
            final DoubleSupplier setpointSupplier
    ) {
        this(
                () -> Phoenix6Utils.latencyCompensateIfSignalIsGood(measurementSignal, measurementVelocitySignal),
                () -> measurementVelocitySignal.refresh().getValue(),
                setpointSupplier
        );
    }

    public boolean isZeroControlEffort() {
        return measurementSupplier.getAsDouble() != setpointSupplier.getAsDouble()
                && measurementVelocitySupplier.getAsDouble() == 0;
    }

    public boolean isAwayFromSetpoint() {
        final double setpoint = setpointSupplier.getAsDouble();
        final double measurement = measurementSupplier.getAsDouble();
        final double measurementVelocity = measurementVelocitySupplier.getAsDouble();

        return setpoint != measurement &&
                (setpoint > measurement ? measurementVelocity < 0 : measurementVelocity > 0);
    }
}
