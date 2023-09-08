package testutils;

import com.ctre.phoenix.CTREJNIWrapper;
import com.ctre.phoenix6.jni.CtreJniWrapper;
import com.revrobotics.jni.RevJNIWrapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.RuntimeLoader;
import frc.robot.utils.closeables.ToClose;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class JNIUtils {
    private static boolean runningHAL = false;
    static {
        if (!ToClose.hasHooked() && !ToClose.hasClosed()) {
            ToClose.hook();
        }
    }

    public static void initializeHAL() {
        if (runningHAL) {
            return;
        }

        assertTrue(HAL.initialize(500, 0));
        ToClose.add(JNIUtils::shutdownHAL);

        runningHAL = true;
    }

    public static void shutdownHAL() {
        if (runningHAL) {
            HAL.shutdown();
            runningHAL = false;
        }
    }

    public static void loadRevJNI() {
        try {
            final RuntimeLoader<RevJNIWrapper> jniLoader = new RuntimeLoader<>(
                    "REVLibDriver", RuntimeLoader.getDefaultExtractionRoot(), RevJNIWrapper.class
            );
            jniLoader.loadLibrary();
        } catch (final IOException ioException) {
            throw new RuntimeException(ioException);
        }
    }

    public static void loadCTREPhoenix5JNI() {
        try {
            final RuntimeLoader<CTREJNIWrapper> jniLoader = new RuntimeLoader<>(
                    "CTRE_PhoenixCCISim", RuntimeLoader.getDefaultExtractionRoot(), CTREJNIWrapper.class
            );
            jniLoader.loadLibrary();
        } catch (final IOException ioException) {
            throw new RuntimeException(ioException);
        }
    }

    public static void loadCTREPhoenix6JNI() {
        try {
            final RuntimeLoader<CtreJniWrapper> jniLoader = new RuntimeLoader<>(
                    "CTRE_PhoenixTools_Sim", RuntimeLoader.getDefaultExtractionRoot(), CtreJniWrapper.class
            );
            jniLoader.loadLibrary();
        } catch (final IOException ioException) {
            throw new RuntimeException(ioException);
        }
    }
}
