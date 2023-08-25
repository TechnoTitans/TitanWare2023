package frc.robot.utils.control;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.Mock;
import org.mockito.Spy;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.stream.Stream;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class PIDUtilsTest {
    @Spy
    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(
            1, 0, 0,
            new TrapezoidProfile.Constraints(1, 1)
    );
    @Mock
    private StatusSignal<Double> positionSignal;
    @Mock
    private StatusSignal<Double> velocitySignal;
    @Mock
    private Timestamp timestamp;

    @ParameterizedTest
    @MethodSource("provideResetProfiledPIDControllerWithStatusSignal")
    void resetProfiledPIDControllerWithStatusSignal(
            final double position,
            final double velocity,
            final double latency
    ) {
        when(positionSignal.refresh()).thenReturn(positionSignal);
        when(velocitySignal.refresh()).thenReturn(velocitySignal);

        when(positionSignal.getError()).thenReturn(StatusCode.OK);
        when(velocitySignal.getError()).thenReturn(StatusCode.OK);

        when(positionSignal.getValue()).thenReturn(position);
        when(velocitySignal.getValue()).thenReturn(velocity);

        when(timestamp.getLatency()).thenReturn(latency);
        when(positionSignal.getTimestamp()).thenReturn(timestamp);

        final double compensatedPosition = position + (velocity * latency);
        PIDUtils.resetProfiledPIDControllerWithStatusSignal(profiledPIDController, positionSignal, velocitySignal);

        verify(profiledPIDController).reset(compensatedPosition, velocity);
    }

    static Stream<Arguments> provideResetProfiledPIDControllerWithStatusSignal() {
        return Stream.of(
                Arguments.of(0, 0, 0),
                Arguments.of(1, 0, 0),
                Arguments.of(0.1683, -0.1524, 0.56),
                Arguments.of(8.9942, 1.454e6, 0.0254),
                Arguments.of(-99.124, 88.8, 0.12),
                Arguments.of(-8e5, 1.4e-2, 1.852e-4)
        );
    }
}