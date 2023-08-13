package frc.robot.utils.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.jni.RevJNIWrapper;
import edu.wpi.first.hal.JNIWrapper;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.net.WPINetJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.RuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.IOException;
import java.util.stream.Stream;

import static com.revrobotics.CANSparkMax.ControlType.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class RevUtilsTest {
    public static final double EPSILON = 1E-6;

    @Mock
    private CANSparkMax sparkMax;

    @BeforeAll
    static void beforeAll() {
        JNIWrapper.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        WPINetJNI.Helper.setExtractOnStaticLoad(false);

        try {
            final RuntimeLoader<RevJNIWrapper> jniLoader = new RuntimeLoader<>(
                    "REVLibDriver", RuntimeLoader.getDefaultExtractionRoot(), RevJNIWrapper.class
            );
            jniLoader.loadLibrary();
        } catch (final IOException ioException) {
            throw new RuntimeException(ioException);
        }
    }

    @ParameterizedTest
    @MethodSource("provideGetSparkMAXMotorVoltage")
    void getSparkMAXMotorVoltage(
            final CANSparkMax.ControlType controlType,
            final double busVoltage,
            final double appliedOutput,
            final double expectedMotorVoltage
    ) {
        when(sparkMax.getBusVoltage()).thenReturn(busVoltage);
        when(sparkMax.getAppliedOutput()).thenReturn(appliedOutput);

        assertEquals(expectedMotorVoltage, RevUtils.getSparkMAXMotorVoltage(sparkMax, controlType), EPSILON);
    }

    private static Stream<Arguments> provideGetSparkMAXMotorVoltage() {
        return Stream.of(
                Arguments.of(kDutyCycle, 12, 1, 12),
                Arguments.of(kDutyCycle, 11.68, 0.5, 5.84),
                Arguments.of(kDutyCycle, 5, -0.5, -2.5),
                Arguments.of(kDutyCycle, 12, -12, -12),
                Arguments.of(kVoltage, 12, 12, 12),
                Arguments.of(kVoltage, 12, -6, -6),
                Arguments.of(kVoltage, 11, 9, 8.25)
        );
    }

    @Test
    void convertControlTypeOutput() {
        when(sparkMax.getBusVoltage()).thenReturn(12d);

        assertEquals(
                RevUtils.convertControlTypeOutput(sparkMax, kDutyCycle, kVoltage, 1), 12,
                EPSILON
        );
        assertEquals(
                RevUtils.convertControlTypeOutput(sparkMax, kDutyCycle, kVoltage, 0.5), 6,
                EPSILON
        );
        assertEquals(
                RevUtils.convertControlTypeOutput(sparkMax, kDutyCycle, kVoltage, -0.75), -9,
                EPSILON
        );
        assertEquals(
                RevUtils.convertControlTypeOutput(sparkMax, kVoltage, kDutyCycle, 12), 1,
                EPSILON
        );
        assertEquals(
                RevUtils.convertControlTypeOutput(sparkMax, kVoltage, kDutyCycle, -11.4), -0.95,
                EPSILON
        );
    }
}