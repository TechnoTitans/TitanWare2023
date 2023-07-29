package frc.robot.utils.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import frc.robot.Constants;
import frc.robot.RobotMap;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

public enum TitanCamera {
    DRIVER_CAM(
            RobotMap.PhotonVision_Driver_Cam,
            new Transform3d(),
            CameraProperties.MICROSOFT_LIFECAM_HD3000,
            true
    ),
    //TODO: put real numbers here -> get these fake intrinsics and distortion values out of here
    PHOTON_FR_APRILTAG_F(
            RobotMap.PhotonVision_FR_Apriltag_F,
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM_F,
            CameraProperties.SPINEL_UC10MPC_ND_OV9281,
            SimCameraProperties.PERFECT_90DEG().getIntrinsics(),
            SimCameraProperties.PERFECT_90DEG().getDistCoeffs(),
            false
    ),
    //TODO: put real numbers here -> get these fake intrinsics and distortion values out of here
    PHOTON_FR_Apriltag_R(
            RobotMap.PhotonVision_FR_Apriltag_R,
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM_R,
            CameraProperties.ARDUCAM_B0332_OV9281,
            SimCameraProperties.PERFECT_90DEG().getIntrinsics(),
            SimCameraProperties.PERFECT_90DEG().getDistCoeffs(),
            false
    ),
    //TODO: put real numbers here -> get these fake intrinsics and distortion values out of here
    PHOTON_FL_Apriltag_L(
            RobotMap.PhotonVision_FL_Apriltag_L,
            Constants.Vision.ROBOT_TO_FL_APRILTAG_CAM_L,
            CameraProperties.ARDUCAM_B0332_OV9281,  //TODO: GET FOR NEW CAMERAS
            SimCameraProperties.PERFECT_90DEG().getIntrinsics(),
            SimCameraProperties.PERFECT_90DEG().getDistCoeffs(),
            false
    ),
    //TODO: put real numbers here -> get these fake intrinsics and distortion values out of here
    PHOTON_BR_Apriltag_B(
            RobotMap.PhotonVision_BR_Apriltag_B,
            Constants.Vision.ROBOT_TO_BR_APRILTAG_CAM_B,
            CameraProperties.ARDUCAM_B0332_OV9281, //TODO: GET FOR NEW CAMERAS
            SimCameraProperties.PERFECT_90DEG().getIntrinsics(),
            SimCameraProperties.PERFECT_90DEG().getDistCoeffs(),
            false
    );

    private final PhotonCamera photonCamera;
    private final Transform3d robotRelativeToCameraTransform;
    private final CameraProperties cameraProperties;
    private final Matrix<N3, N3> instrinsicsMatrix;
    private final Matrix<N5, N1> distortionMatrix;
    private final boolean driverCam;

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final Matrix<N3, N3> instrinsicsMatrix,
            final Matrix<N5, N1> distortionMatrix,
            final boolean driverCam
    ) {
        this.photonCamera = new PhotonCamera(photonCameraName);
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.cameraProperties = cameraProperties;
        this.instrinsicsMatrix = instrinsicsMatrix;
        this.distortionMatrix = distortionMatrix;
        this.driverCam = driverCam;

        this.photonCamera.setDriverMode(driverCam);
    }

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final boolean driverCam
    ) {
        this(
                photonCameraName,
                robotRelativeToCameraTransform,
                cameraProperties,
                SimCameraProperties.PERFECT_90DEG().getIntrinsics(),
                SimCameraProperties.PERFECT_90DEG().getDistCoeffs(),
                driverCam
        );
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    public Transform3d getRobotRelativeToCameraTransform() {
        return robotRelativeToCameraTransform;
    }

    public CameraProperties getCameraProperties() {
        return cameraProperties;
    }

    public Matrix<N3, N3> getInstrinsicsMatrix() {
        return instrinsicsMatrix;
    }

    public Matrix<N5, N1> getDistortionMatrix() {
        return distortionMatrix;
    }

    public boolean isDriverCam() {
        return driverCam;
    }

    public SimCameraProperties toSimCameraProperties() {
        final CameraProperties properties = getCameraProperties();
        final SimCameraProperties simCameraProperties = new SimCameraProperties();

        simCameraProperties.setCalibration(
                properties.getCamResolutionWidthPx(),
                properties.getCamResolutionHeightPx(),
                getInstrinsicsMatrix(),
                getDistortionMatrix()
        );

        simCameraProperties.setFPS(properties.getAvgFPS());
        return simCameraProperties;
    }
}
