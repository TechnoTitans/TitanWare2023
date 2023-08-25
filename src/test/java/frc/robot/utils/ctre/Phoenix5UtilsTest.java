package frc.robot.utils.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class Phoenix5UtilsTest {
    private static final double EPSILON = 1E-7;

    @ParameterizedTest
    @MethodSource("provideRotationsToCTREPhoenix5NativeUnits")
    void rotationsToCTREPhoenix5NativeUnits(final double rotations, final int expectedNativeUnits) {
        assertEquals(expectedNativeUnits, Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(rotations));
    }

    static Stream<Arguments> provideRotationsToCTREPhoenix5NativeUnits() {
        return Stream.of(
                Arguments.of(1, 4096),
                Arguments.of(0.5, 2048),
                Arguments.of(-0.5, -2048),
                Arguments.of(247.248, 1012727),
                Arguments.of(-9.65, -39526),
                Arguments.of(0, 0)
        );
    }

    @Test
    void getPhoenix6To5ControlInput() {
        assertEquals(1, Phoenix5Utils.getPhoenix6To5ControlInput(ControlMode.PercentOutput, 1), EPSILON);
        assertEquals(4096, Phoenix5Utils.getPhoenix6To5ControlInput(ControlMode.Position, 1), EPSILON);
        assertEquals(
                -2048, Phoenix5Utils.getPhoenix6To5ControlInput(ControlMode.Position, -0.5), EPSILON
        );
        assertEquals(
                19154, Phoenix5Utils.getPhoenix6To5ControlInput(ControlMode.Position, 4.6765), EPSILON
        );

        assertEquals(3072, Phoenix5Utils.getPhoenix6To5ControlInput(ControlMode.Velocity, 0.75), EPSILON);
        assertEquals(
                -11825, Phoenix5Utils.getPhoenix6To5ControlInput(ControlMode.Velocity, -2.887), EPSILON
        );
    }
}