package frc.robot.utils.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;

import java.io.IOException;

public record TitanCamera(
        PhotonCamera photonCamera,
        Transform3d robotRelativeToCameraTransform,
        double camDiagonalFOVDeg,
        int camResolutionWidthPx,
        int camResolutionHeightPx,
        double minTargetArea,
        boolean driverCam
) {
    public static final AprilTagFieldLayout apriltagFieldLayout;

    static {
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException ioException) {
            layout = null;
            DriverStation.reportError("Failed to load AprilTagFieldLayout", ioException.getStackTrace());
        }

        apriltagFieldLayout = layout;
        if (apriltagFieldLayout != null) {
            //TODO: do we need setOrigin
            apriltagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    public TitanCamera(
            final PhotonCamera photonCamera,
            final Transform3d robotRelativeToCameraTransform,
            final double camDiagonalFOVDeg,
            final int camResolutionWidthPx,
            final int camResolutionHeightPx,
            final double minTargetArea,
            final boolean driverCam
    ) {
        this.photonCamera = photonCamera;
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.camDiagonalFOVDeg = camDiagonalFOVDeg;
        this.camResolutionWidthPx = camResolutionWidthPx;
        this.camResolutionHeightPx = camResolutionHeightPx;
        this.minTargetArea = minTargetArea;
        this.driverCam = driverCam;

        this.photonCamera.setDriverMode(driverCam);
    }

    public TitanCamera(final String photonCameraName,
                       final Transform3d robotRelativeToCameraTransform,
                       final double camDiagonalFOVDeg,
                       final int camResolutionWidthPx,
                       final int camResolutionHeightPx,
                       final double minTargetArea,
                       final boolean driverCam) {
        this(
                new PhotonCamera(photonCameraName),
                robotRelativeToCameraTransform,
                camDiagonalFOVDeg,
                camResolutionWidthPx,
                camResolutionHeightPx,
                minTargetArea,
                driverCam
        );
    }

    public TitanCamera(
            final PhotonCamera photonCamera,
            final Transform3d robotRelativeToCameraTransform,
            final double camDiagonalFOVDeg,
            final int camResolutionWidthPx,
            final int camResolutionHeightPx,
            final double minTargetArea
    ) {
        this(
                photonCamera,
                robotRelativeToCameraTransform,
                camDiagonalFOVDeg,
                camResolutionWidthPx,
                camResolutionHeightPx,
                minTargetArea,
                false
        );
    }

    public TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final boolean driverCam) {
        this(
                new PhotonCamera(photonCameraName),
                robotRelativeToCameraTransform,
                cameraProperties.getCamDiagonalFOVDeg(),
                cameraProperties.getCamResolutionWidthPx(),
                cameraProperties.getCamResolutionHeightPx(),
                cameraProperties.getMinTargetArea(),
                driverCam
        );
    }

    public TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties) {
        this(
                new PhotonCamera(photonCameraName),
                robotRelativeToCameraTransform,
                cameraProperties.getCamDiagonalFOVDeg(),
                cameraProperties.getCamResolutionWidthPx(),
                cameraProperties.getCamResolutionHeightPx(),
                cameraProperties.getMinTargetArea()
        );
    }
}
