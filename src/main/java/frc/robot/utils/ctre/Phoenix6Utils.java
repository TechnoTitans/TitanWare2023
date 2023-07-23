package frc.robot.utils.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class Phoenix6Utils {
    /**
     * Performs latency compensation
     * (using {@link BaseStatusSignal#getLatencyCompensatedValue(StatusSignal, StatusSignal)}) on {@link StatusSignal}s
     * <p>
     * Only compensates if the {@link com.ctre.phoenix6.StatusCode} of the {@link StatusSignal}
     * is OK ({@link StatusCode#isOK()})
     * <p>
     * If the {@link StatusCode} is not OK, then it just returns the signal value without latency compensation
     * @param signal the {@link StatusSignal}, ex. Position
     * @param deltaSignal the delta/derivative of the {@link StatusSignal}, ex. Velocity
     * @return the latency compensated value
     * @see StatusSignal
     * @see StatusCode
     * @see BaseStatusSignal#getLatencyCompensatedValue(StatusSignal, StatusSignal)
     */
    public static double latencyCompensateIfSignalIsGood(
            final StatusSignal<Double> signal,
            final StatusSignal<Double> deltaSignal
    ) {
        final StatusSignal<Double> refreshedSignal = signal.refresh();
        final StatusSignal<Double> refreshedDeltaSignal = deltaSignal.refresh();

        //TODO: is checking isOK ok here? should we instead just check for StatusCode.CanMessageStale?
        if (refreshedSignal.getError().isOK() && refreshedDeltaSignal.getError().isOK()) {
            return BaseStatusSignal.getLatencyCompensatedValue(
                    refreshedSignal,
                    refreshedDeltaSignal
            );
        } else {
            return signal.getValue();
        }
    }
}
