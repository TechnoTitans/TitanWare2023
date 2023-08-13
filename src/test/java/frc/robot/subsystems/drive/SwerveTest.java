package frc.robot.subsystems.drive;

import com.ctre.phoenix.CTREJNIWrapper;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.RuntimeLoader;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

@ExtendWith(MockitoExtension.class)
class SwerveTest {
    @Mock
    private Swerve mockedSwerve;

    @BeforeAll
    static void beforeAll() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void periodic() {
    }

    @Test
    void getGyro() {

    }

    @Test
    void getPitch() {
    }

    @Test
    void getRoll() {
    }

    @Test
    void getYaw() {
    }

    @Test
    void setAngle() {
    }

    @Test
    void zeroRotation() {
    }

    @Test
    void getRobotRelativeSpeeds() {
    }

    @Test
    void getFieldRelativeSpeeds() {
    }

    @Test
    void getModuleStates() {
    }

    @Test
    void getModuleLastDesiredStates() {
    }

    @Test
    void getModulePositions() {
    }

    @Test
    void drive() {
    }

    @Test
    void testDrive() {
    }

    @Test
    void testDrive1() {
    }

    @Test
    void stop() {
    }

    @Test
    void faceDirection() {
    }

    @Test
    void testFaceDirection() {
    }

    @Test
    void rawSet() {
    }

    @Test
    void zero() {
    }

    @Test
    void wheelX() {
    }

    @Test
    void setNeutralMode() {
    }
}