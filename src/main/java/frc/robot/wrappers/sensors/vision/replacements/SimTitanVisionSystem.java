package frc.robot.wrappers.sensors.vision.replacements;

import frc.robot.utils.vision.TitanCamera;
import org.photonvision.SimVisionSystem;

import java.lang.reflect.Field;

public class SimTitanVisionSystem extends SimVisionSystem {
    public static final double NO_LED_MAX_LED_RANGE = 1000;

    private final TitanCamera titanCamera;
    private final SimTitanCamera simTitanCamera;

    public SimTitanVisionSystem(
            final TitanCamera titanCamera,
            final SimTitanCamera simTitanCamera
    ) {
        super(
                titanCamera.getPhotonCamera().getName(),
                titanCamera.getCamDiagonalFOVDeg(),
                titanCamera.getRobotRelativeToCameraTransform(),
                NO_LED_MAX_LED_RANGE,
                titanCamera.getCamResolutionWidthPx(),
                titanCamera.getCamResolutionHeightPx(),
                titanCamera.getMinTargetArea()
        );

        this.titanCamera = titanCamera;
        this.simTitanCamera = simTitanCamera;

        try {
            final Field simPhotonCamField = super.getClass().getSuperclass().getDeclaredField("cam");
            simPhotonCamField.setAccessible(true);
            simPhotonCamField.set(this, simTitanCamera);
        } catch (final NoSuchFieldException | IllegalAccessException reflectiveOperationException) {
            // no need to do anything special here, we are already in sim, so we don't really care
            // just go ahead and rethrow it
            throw new RuntimeException(reflectiveOperationException);
        } catch (final IllegalArgumentException illegalArgumentException) {
            // this is also likely a bug, so just rethrow
            throw new RuntimeException(illegalArgumentException);
        }
    }

    public SimTitanVisionSystem(final TitanCamera titanCamera) {
        this(titanCamera, new SimTitanCamera(titanCamera));
    }
}
