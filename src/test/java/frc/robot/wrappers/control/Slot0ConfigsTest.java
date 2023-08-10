package frc.robot.wrappers.control;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class Slot0ConfigsTest {
    @Test
    void testConformity() {
        final double kP = 1;
        final double kI = 0.2974;
        final double kD = 0.9999;
        final double kS = 0.254;
        final double kV = 1.683;

        final com.ctre.phoenix6.configs.Slot0Configs phoenixSlot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        phoenixSlot0Configs.kP = kP;
        phoenixSlot0Configs.kI = kI;
        phoenixSlot0Configs.kD = kD;
        phoenixSlot0Configs.kS = kS;
        phoenixSlot0Configs.kV = kV;

        final frc.robot.wrappers.control.Slot0Configs wrappedSlot0Configs = new Slot0Configs(kP, kD, kS, kV);
        assertEquals(phoenixSlot0Configs.kP, wrappedSlot0Configs.kP);
        assertEquals(phoenixSlot0Configs.kD, wrappedSlot0Configs.kD);
        assertEquals(phoenixSlot0Configs.kS, wrappedSlot0Configs.kS);
        assertEquals(phoenixSlot0Configs.kV, wrappedSlot0Configs.kV);
    }
}