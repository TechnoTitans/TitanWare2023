package frc.robot.utils.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotMap;
import frc.robot.constants.Constants;
import org.photonvision.PhotonCamera;

public enum TitanCamera {
    DRIVER_CAM(
            RobotMap.PhotonVision_Driver_Cam,
            new Transform3d(),
            true
    ),
    PHOTON_FR_Apriltag_F(
            RobotMap.PhotonVision_FR_Apriltag_F,
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM_F,
            false
    ),
    PHOTON_FR_Apriltag_R(
            RobotMap.PhotonVision_FR_Apriltag_R,
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM_R,
            false
    ),
    PHOTON_FL_Apriltag_L(
            RobotMap.PhotonVision_FL_Apriltag_L,
            Constants.Vision.ROBOT_TO_FL_APRILTAG_CAM_L,
            false
    ),
    PHOTON_BR_Apriltag_B(
            RobotMap.PhotonVision_BR_Apriltag_B,
            Constants.Vision.ROBOT_TO_BR_APRILTAG_CAM_B,
            false
    );

    private final PhotonCamera photonCamera;
    private final Transform3d robotRelativeToCameraTransform;
    private final boolean driverCam;

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final boolean driverCam
    ) {
        this.photonCamera = new PhotonCamera(photonCameraName);
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.driverCam = driverCam;

        this.photonCamera.setDriverMode(driverCam);
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    public Transform3d getRobotRelativeToCameraTransform() {
        return robotRelativeToCameraTransform;
    }

    public boolean isDriverCam() {
        return driverCam;
    }
}
