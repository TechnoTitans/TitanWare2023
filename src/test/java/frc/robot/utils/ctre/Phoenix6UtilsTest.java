package frc.robot.utils.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
class Phoenix6UtilsTest {
    private static final double EPSILON = 1E-7;

    @Mock
    private StatusSignal<Double> positionSignal;
    @Mock
    private StatusSignal<Double> velocitySignal;
    @Mock
    private Timestamp timestamp;

    @ParameterizedTest
    @MethodSource("provideLatencyCompensateIfSignalIsGood")
    void latencyCompensateIfSignalIsGood(
            final double position,
            final double velocity,
            final double latency,
            final double expected
    ) {
        when(positionSignal.refresh()).thenReturn(positionSignal);
        when(velocitySignal.refresh()).thenReturn(velocitySignal);

        when(positionSignal.getError()).thenReturn(StatusCode.OK);
        when(velocitySignal.getError()).thenReturn(StatusCode.OK);

        when(positionSignal.getValue()).thenReturn(position);
        when(velocitySignal.getValue()).thenReturn(velocity);

        when(timestamp.getLatency()).thenReturn(latency);
        when(positionSignal.getTimestamp()).thenReturn(timestamp);

        assertEquals(
                expected, Phoenix6Utils.latencyCompensateIfSignalIsGood(positionSignal, velocitySignal), EPSILON
        );
    }

    static Stream<Arguments> provideLatencyCompensateIfSignalIsGood() {
        return Stream.of(
                Arguments.of(0, 0, 0, 0),
                Arguments.of(1, 0, 0, 1),
                Arguments.of(1, 1, 0, 1),
                Arguments.of(1, 1, 1, 2),
                Arguments.of(0.7562, 0.8274, 0.924, 1.5207176),
                Arguments.of(-8.12842, 1.2845, 0.08274, -8.02214047),
                Arguments.of(1e6, -1e12, 1e-9, 999000)
        );
    }

    @ParameterizedTest
    @MethodSource("latencyCompensateIfSignalIsGood_SignalBad")
    void latencyCompensateIfSignalIsGood_ShouldNotCompensateIfPositionSignalIsBad(
            final double position,
            final StatusCode badStatusCode
    ) {
        if (badStatusCode.isOK()) {
            throw new IllegalArgumentException("statusCode should not be OK!");
        }

        when(positionSignal.refresh()).thenReturn(positionSignal);
        when(velocitySignal.refresh()).thenReturn(velocitySignal);

        when(positionSignal.getError()).thenReturn(badStatusCode);
        when(positionSignal.getValue()).thenReturn(position);

        assertEquals(
                position, Phoenix6Utils.latencyCompensateIfSignalIsGood(positionSignal, velocitySignal)
        );
    }

    @ParameterizedTest
    @MethodSource("latencyCompensateIfSignalIsGood_SignalBad")
    void latencyCompensateIfSignalIsGood_ShouldNotCompensateIfVelocitySignalIsBad(
            final double position,
            final StatusCode badStatusCode
    ) {
        if (badStatusCode.isOK()) {
            throw new IllegalArgumentException("statusCode should not be OK!");
        }

        when(positionSignal.refresh()).thenReturn(positionSignal);
        when(velocitySignal.refresh()).thenReturn(velocitySignal);

        when(positionSignal.getError()).thenReturn(StatusCode.OK);
        when(velocitySignal.getError()).thenReturn(badStatusCode);

        when(positionSignal.getValue()).thenReturn(position);

        assertEquals(
                position, Phoenix6Utils.latencyCompensateIfSignalIsGood(positionSignal, velocitySignal)
        );
    }

    static Stream<Arguments> latencyCompensateIfSignalIsGood_SignalBad() {
        return Stream.of(
                Arguments.of(0, StatusCode.CanMessageStale),
                Arguments.of(1, StatusCode.TxFailed),
                Arguments.of(927.7816, StatusCode.RxTimeout),
                Arguments.of(-24.67, StatusCode.CanOverflowed),
                Arguments.of(1.85e-6, StatusCode.BufferFailure)
        );
    }
}