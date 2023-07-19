package frc.robot.utils.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;

import java.io.IOException;

public record TitanCamera(PhotonCamera photonCamera, Transform3d robotRelativeToCameraTransform, boolean driverCam) {
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
            final boolean driverCam) {
        this.photonCamera = photonCamera;
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.driverCam = driverCam;

        this.photonCamera.setDriverMode(driverCam);
    }

    public TitanCamera(final String photonCameraName, final Transform3d robotRelativeToCameraTransform, final boolean driverCam) {
        this(new PhotonCamera(photonCameraName), robotRelativeToCameraTransform, driverCam);
    }

    public TitanCamera(final PhotonCamera photonCamera, final Transform3d robotRelativeToCameraTransform) {
        this(photonCamera, robotRelativeToCameraTransform, false);
    }

    public TitanCamera(final String photonCameraName, final Transform3d robotRelativeToCameraTransform) {
        this(new PhotonCamera(photonCameraName), robotRelativeToCameraTransform);
    }
}
