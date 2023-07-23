package frc.robot.utils.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotMap;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonCamera;

import java.io.IOException;

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
            new Matrix<>(new SimpleMatrix(new double[][] {
                    {6.5746697944293521e+002, 0, 3.1950000000000000e+002},
                    {0, 6.5746697944293521e+002, 2.3950000000000000e+002},
                    {0, 0, 1}
            })),
            new Matrix<>(new SimpleMatrix(new double[][] {
                    {-4.1802327176423804e-001, 5.0715244063187526e-001, 0, 0, -5.7843597214487474e-001},
            }))
    ),
    //TODO: put real numbers here -> get these fake intrinsics and distortion values out of here
    PHOTON_FR_Apriltag_R(
            RobotMap.PhotonVision_FR_Apriltag_R,
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM_R,
            CameraProperties.ARDUCAM_B0332_OV9281,
            new Matrix<>(new SimpleMatrix(new double[][] {
                    {6.5746697944293521e+002, 0, 3.1950000000000000e+002},
                    {0, 6.5746697944293521e+002, 2.3950000000000000e+002},
                    {0, 0, 1}
            })),
            new Matrix<>(new SimpleMatrix(new double[][] {
                    {-4.1802327176423804e-001, 5.0715244063187526e-001, 0, 0, -5.7843597214487474e-001},
            }))
    );
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

    private final PhotonCamera photonCamera;
    private final Transform3d robotRelativeToCameraTransform;
    private final double camDiagonalFOVDeg;
    private final int camResolutionWidthPx;
    private final int camResolutionHeightPx;
    private final double minTargetArea;
    private final Matrix<N3, N3> instrinsicsMatrix;
    private final Matrix<N5, N1> distortionMatrix;
    private final boolean driverCam;

    TitanCamera(
            final PhotonCamera photonCamera,
            final Transform3d robotRelativeToCameraTransform,
            final double camDiagonalFOVDeg,
            final int camResolutionWidthPx,
            final int camResolutionHeightPx,
            final double minTargetArea,
            final Matrix<N3, N3> instrinsicsMatrix,
            final Matrix<N5, N1> distortionMatrix,
            final boolean driverCam
    ) {
        this.photonCamera = photonCamera;
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.camDiagonalFOVDeg = camDiagonalFOVDeg;
        this.camResolutionWidthPx = camResolutionWidthPx;
        this.camResolutionHeightPx = camResolutionHeightPx;
        this.minTargetArea = minTargetArea;
        this.instrinsicsMatrix = instrinsicsMatrix;
        this.distortionMatrix = distortionMatrix;
        this.driverCam = driverCam;

        this.photonCamera.setDriverMode(driverCam);
    }

    TitanCamera(final String photonCameraName,
                       final Transform3d robotRelativeToCameraTransform,
                       final double camDiagonalFOVDeg,
                       final int camResolutionWidthPx,
                       final int camResolutionHeightPx,
                       final double minTargetArea,
                       final Matrix<N3, N3> instrinsicsMatrix,
                       final Matrix<N5, N1> distortionMatrix,
                       final boolean driverCam
    ) {
        this(
                new PhotonCamera(photonCameraName),
                robotRelativeToCameraTransform,
                camDiagonalFOVDeg,
                camResolutionWidthPx,
                camResolutionHeightPx,
                minTargetArea,
                instrinsicsMatrix,
                distortionMatrix,
                driverCam
        );
    }

    TitanCamera(
            final PhotonCamera photonCamera,
            final Transform3d robotRelativeToCameraTransform,
            final double camDiagonalFOVDeg,
            final int camResolutionWidthPx,
            final int camResolutionHeightPx,
            final double minTargetArea,
            final Matrix<N3, N3> instrinsicsMatrix,
            final Matrix<N5, N1> distortionMatrix
    ) {
        this(
                photonCamera,
                robotRelativeToCameraTransform,
                camDiagonalFOVDeg,
                camResolutionWidthPx,
                camResolutionHeightPx,
                minTargetArea,
                instrinsicsMatrix,
                distortionMatrix,
                false
        );
    }

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final Matrix<N3, N3> instrinsicsMatrix,
            final Matrix<N5, N1> distortionMatrix,
            final boolean driverCam
    ) {
        this(
                new PhotonCamera(photonCameraName),
                robotRelativeToCameraTransform,
                cameraProperties.getCamDiagonalFOVDeg(),
                cameraProperties.getCamResolutionWidthPx(),
                cameraProperties.getCamResolutionHeightPx(),
                cameraProperties.getMinTargetArea(),
                instrinsicsMatrix,
                distortionMatrix,
                driverCam
        );
    }

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final boolean driverCam
    ) {
        this(
                new PhotonCamera(photonCameraName),
                robotRelativeToCameraTransform,
                cameraProperties.getCamDiagonalFOVDeg(),
                cameraProperties.getCamResolutionWidthPx(),
                cameraProperties.getCamResolutionHeightPx(),
                cameraProperties.getMinTargetArea(),
                new Matrix<>(Nat.N3(), Nat.N3()),
                new Matrix<>(Nat.N5(), Nat.N1()),
                driverCam
        );
    }

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final Matrix<N3, N3> instrinsicsMatrix,
            final Matrix<N5, N1> distortionMatrix
    ) {
        this(
                new PhotonCamera(photonCameraName),
                robotRelativeToCameraTransform,
                cameraProperties.getCamDiagonalFOVDeg(),
                cameraProperties.getCamResolutionWidthPx(),
                cameraProperties.getCamResolutionHeightPx(),
                cameraProperties.getMinTargetArea(),
                instrinsicsMatrix,
                distortionMatrix
        );
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    public Transform3d getRobotRelativeToCameraTransform() {
        return robotRelativeToCameraTransform;
    }

    public double getCamDiagonalFOVDeg() {
        return camDiagonalFOVDeg;
    }

    public int getCamResolutionWidthPx() {
        return camResolutionWidthPx;
    }

    public int getCamResolutionHeightPx() {
        return camResolutionHeightPx;
    }

    public double getMinTargetArea() {
        return minTargetArea;
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
}
