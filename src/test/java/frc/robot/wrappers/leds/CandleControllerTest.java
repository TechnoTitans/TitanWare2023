package frc.robot.wrappers.leds;

import com.ctre.phoenix.CTREJNIWrapper;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.RuntimeLoader;
import frc.robot.utils.SuperstructureStates;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class CandleControllerTest {
    @Mock
    private CANdle mockedCandle;

    @BeforeAll
    static void beforeAll() {
        assertTrue(HAL.initialize(500, 0));
        try {
            final RuntimeLoader<CTREJNIWrapper> jniLoader = new RuntimeLoader<>(
                    "CTRE_PhoenixCCISim", RuntimeLoader.getDefaultExtractionRoot(), CTREJNIWrapper.class
            );
            jniLoader.loadLibrary();
        } catch (final IOException ioException) {
            throw new RuntimeException(ioException);
        }
    }

    @Test
    void setAndGetState() {
        final CandleController controller = new CandleController(mockedCandle);
        when(mockedCandle.setLEDs(anyInt(), anyInt(), anyInt())).thenReturn(ErrorCode.OK);

        final SuperstructureStates.CANdleState candlePurple = SuperstructureStates.CANdleState.PURPLE;
        final SuperstructureStates.CANdleState candleYellow = SuperstructureStates.CANdleState.YELLOW;
        final SuperstructureStates.CANdleState candleOff = SuperstructureStates.CANdleState.OFF;

        controller.setState(candlePurple);
        assertEquals(controller.getCurrentState(), candlePurple);

        controller.setState(candleYellow);
        assertEquals(controller.getCurrentState(), candleYellow);

        controller.setState(candleOff);
        assertEquals(controller.getCurrentState(), candleOff);

        verify(mockedCandle).setLEDs(candlePurple.getR(), candlePurple.getG(), candlePurple.getB());
        verify(mockedCandle).setLEDs(candleYellow.getR(), candleYellow.getG(), candleYellow.getB());
        verify(mockedCandle).setLEDs(candleOff.getR(), candleOff.getG(), candleOff.getB());

        verifyNoMoreInteractions(mockedCandle);
    }

    @Test
    void setStrobe() {
        // TODO: test this?
    }
}