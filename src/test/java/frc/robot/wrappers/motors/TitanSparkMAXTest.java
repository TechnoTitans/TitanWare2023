package frc.robot.wrappers.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Spy;
import org.mockito.junit.jupiter.MockitoExtension;
import testutils.JNIUtils;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class TitanSparkMAXTest {
    @Spy
    private final TitanSparkMAX titanSparkMAX = new TitanSparkMAX(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    @Spy
    final SparkMaxPIDController sparkMaxPIDController = titanSparkMAX.getPIDController();

    @BeforeAll
    static void beforeAll() {
        JNIUtils.initializeHAL();
        JNIUtils.loadRevJNI();
    }

    @AfterEach
    void tearDown() {
        if (!titanSparkMAX.isClosed()) {
            titanSparkMAX.close();
        }
    }

    @Test
    void set() {
        doReturn(sparkMaxPIDController).when(titanSparkMAX).getPIDController();

        titanSparkMAX.set(CANSparkMax.ControlType.kDutyCycle, 1);
        verify(titanSparkMAX).getPIDController();
        verify(sparkMaxPIDController).setReference(1, CANSparkMax.ControlType.kDutyCycle);

        titanSparkMAX.set(CANSparkMax.ControlType.kVoltage, 12.61);
        verify(titanSparkMAX, times(2)).getPIDController();
        verify(sparkMaxPIDController).setReference(12.61, CANSparkMax.ControlType.kVoltage);

        titanSparkMAX.set(CANSparkMax.ControlType.kPosition, 0.56);
        verify(titanSparkMAX, times(3)).getPIDController();
        verify(sparkMaxPIDController).setReference(0.56, CANSparkMax.ControlType.kPosition);
    }

    @Test
    void setSimFreeSpeed() {
        // cannot mock native static methods, so impossible to test a legitimate call to setSimFreeSpeed since it
        // invokes the CANSparkMaxJNI

        titanSparkMAX.close();
        assertThrowsExactly(IllegalStateException.class, () -> titanSparkMAX.setSimFreeSpeed(0));
    }

    @Test
    void setSimStallTorque() {
        // cannot mock native static methods, so impossible to test a legitimate call to setSimStallTorque since it
        // invokes the CANSparkMaxJNI

        titanSparkMAX.close();
        assertThrowsExactly(IllegalStateException.class, () -> titanSparkMAX.setSimStallTorque(0));
    }

    @Test
    void isClosed() {
        assertFalse(titanSparkMAX.isClosed());
        titanSparkMAX.close();
        assertTrue(titanSparkMAX.isClosed());
    }
}