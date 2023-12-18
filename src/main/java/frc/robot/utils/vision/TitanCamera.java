package frc.robot.utils.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotMap;
import frc.robot.constants.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

import java.util.List;

public enum TitanCamera {
    DRIVER_CAM(
            RobotMap.PhotonVision_Driver_Cam,
            new Transform3d(),
            CameraProperties.MICROSOFT_LIFECAM_HD3000,
            true
    ),
    PHOTON_FR_Apriltag_F(
            RobotMap.PhotonVision_FR_Apriltag_F,
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM_F,
            CameraProperties.SPINEL_UC10MPC_ND_OV9281,
            new TitanCameraCalibration()
                    .withCalibrations(
                            CameraProperties.Resolution.R1280x720,
                            MatBuilder.fill(Nat.N3(), Nat.N3(),
                                    743.6218079385169, 0.0, 674.3875273200663,
                                    0.0, 747.5846567641577, 447.6187926456991,
                                    0.0, 0.0, 1.0
                            ),
                            VecBuilder.fill(
                                    0.1591471755951106,
                                    -0.2841112431032636,
                                    5.712961256316108E-4,
                                    -0.00491917187052816,
                                    0.11475210467385825
                            ),
                            List.of(
                                    0.26413929685366616,
                                    0.24672891266691466,
                                    0.24742156205490679,
                                    0.35897103874327424,
                                    0.18798008099384472,
                                    0.2574947657669096,
                                    0.2543343831955835,
                                    0.1901845438760257,
                                    0.1726084898602154,
                                    0.23101658327922658,
                                    0.146508295309592,
                                    0.8194278741715146,
                                    0.20775031023857174,
                                    0.28835833700178876,
                                    0.22680708898063418,
                                    0.1402878473026035,
                                    0.21791505812283923
                            ),
                            0.14856507728397927
                    ),
            false
    ),
    PHOTON_FR_Apriltag_R(
            RobotMap.PhotonVision_FR_Apriltag_R,
            Constants.Vision.ROBOT_TO_FR_APRILTAG_CAM_R,
            CameraProperties.ARDUCAM_B0332_OV9281,
            new TitanCameraCalibration()
                    .withCalibrations(
                            CameraProperties.Resolution.R1280x800,
                            MatBuilder.fill(Nat.N3(), Nat.N3(),
                                    894.2562605297965, 0.0, 639.9829530010343,
                                    0.0, 892.6042435867523, 411.79669510853427,
                                    0.0, 0.0, 1.0
                            ),
                            VecBuilder.fill(
                                    0.15397416138064096,
                                    -0.28355930665899937,
                                    0.0014608161467837982,
                                    -0.0014540261991797222,
                                    0.11406457397229057
                            ),
                            List.of(
                                    0.17553998250675493,
                                    0.10508370659986599,
                                    0.13132026974437952,
                                    0.2647536868296292,
                                    0.23229163028573407,
                                    0.11706132534831985,
                                    0.12144872106437204,
                                    0.13590267632147376,
                                    0.11841807877295425,
                                    0.1310955489609727,
                                    0.1163617355336164,
                                    0.12927240367590576,
                                    0.33126115578019744,
                                    0.28894560190347074,
                                    0.2173528212321558,
                                    0.19248955448372637,
                                    0.23568974445815613,
                                    0.2232293112265309,
                                    0.18728524770526037,
                                    0.1647013928030047
                            ),
                            0.06387463714283612
                    )
                    .withCalibrations(
                            CameraProperties.Resolution.R800x600,
                            MatBuilder.fill(Nat.N3(), Nat.N3(),
                                    683.201957503352, 0.0, 408.18115707888694,
                                    0.0, 681.5900397776495, 321.5456475271882,
                                    0.0, 0.0, 1.0
                            ),
                            VecBuilder.fill(
                                    0.15397416138064096,
                                    -0.28355930665899937,
                                    0.0014608161467837982,
                                    -0.0014540261991797222,
                                    0.11406457397229057
                            ),
                            List.of(
                                    0.1348343420598462,
                                    0.15162216102087045,
                                    0.14311913839062548,
                                    0.10304150221376314,
                                    0.12717932783634384,
                                    0.17348749900940597,
                                    0.13390153820811249,
                                    0.1202435741961856,
                                    0.2090948148572528,
                                    0.19253054784803,
                                    0.12287332983893427,
                                    0.15219680128567908,
                                    0.1598936620187205,
                                    0.1607970815392953
                            ),
                            0.027956401156254623
                    )
                    .withCalibrations(
                            CameraProperties.Resolution.R1280x720,
                            MatBuilder.fill(Nat.N3(), Nat.N3(),
                                    908.708184379716, 0.0, 623.8361593082838,
                                    0.0, 913.1020519669654, 371.5509219532872,
                                    0.0, 0.0, 1.0
                            ),
                            VecBuilder.fill(
                                    0.061027763093237396,
                                    -0.09831807768480012,
                                    6.334057731293919E-4,
                                    -0.0077648636605850896,
                                    0.02571207764869836
                            ),
                            List.of(
                                    0.2273379456568952,
                                    0.09973719269174569,
                                    0.10018417557561582,
                                    0.12256105918951435,
                                    0.1130820944649044,
                                    0.14139529367297488,
                                    0.13231946242809245,
                                    0.20890981459246302,
                                    0.19477193605658355,
                                    0.12568109152536117,
                                    0.12386713513852286,
                                    0.2829514383115909,
                                    0.07560381533038615,
                                    0.12492815071040511,
                                    0.14071171707421307,
                                    0.25486449986418086,
                                    0.14513214460362583
                            ),
                            0.05685150616452978
                    ),
            false
    ),
    //TODO: put real numbers here -> do not use PERFECT_90DEG
    PHOTON_FL_Apriltag_L(
            RobotMap.PhotonVision_FL_Apriltag_L,
            Constants.Vision.ROBOT_TO_FL_APRILTAG_CAM_L,
            CameraProperties.PERFECT_90,  //TODO: GET FOR NEW CAMERAS
            TitanCameraCalibration.fromSimCameraProperties(SimCameraProperties.PERFECT_90DEG()),
            false
    ),
    //TODO: put real numbers here -> do not use PERFECT_90DEG
    PHOTON_BR_Apriltag_B(
            RobotMap.PhotonVision_BR_Apriltag_B,
            Constants.Vision.ROBOT_TO_BR_APRILTAG_CAM_B,
            CameraProperties.PERFECT_90, //TODO: GET FOR NEW CAMERAS
            TitanCameraCalibration.fromSimCameraProperties(SimCameraProperties.PERFECT_90DEG()),
            false
    );

    private final PhotonCamera photonCamera;
    private final Transform3d robotRelativeToCameraTransform;
    private final CameraProperties cameraProperties;
    private final TitanCameraCalibration cameraCalibration;
    private final boolean driverCam;

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotRelativeToCameraTransform,
            final CameraProperties cameraProperties,
            final TitanCameraCalibration titanCameraCalibration,
            final boolean driverCam
    ) {
        this.photonCamera = new PhotonCamera(photonCameraName);
        this.robotRelativeToCameraTransform = robotRelativeToCameraTransform;
        this.cameraProperties = cameraProperties;
        this.cameraCalibration = titanCameraCalibration;
        this.driverCam = driverCam;

        // if it isn't a driverCam, then it should have proper calibration data
        if (!driverCam) {
            for (final CameraProperties.Resolution resolution : cameraProperties.getResolutions()) {
                if (!cameraCalibration.hasResolution(resolution)) {
                    throw new RuntimeException(
                            String.format(
                                    "Camera %s(%s) does not have calibration data for specified %s",
                                    photonCameraName, cameraProperties, resolution
                            )
                    );
                }
            }
        }


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
                TitanCameraCalibration.fromSimCameraProperties(SimCameraProperties.PERFECT_90DEG()),
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

    public TitanCameraCalibration getCameraCalibration() {
        return cameraCalibration;
    }

    public boolean isDriverCam() {
        return driverCam;
    }

    public SimCameraProperties toSimCameraProperties(final CameraProperties.Resolution resolution) {
        final SimCameraProperties simCameraProperties = cameraCalibration.getSimCameraProperties(resolution);
        simCameraProperties.setFPS(cameraProperties.getAvgFPS());

        return simCameraProperties;
    }

    public SimCameraProperties toSimCameraProperties() {
        return toSimCameraProperties(cameraProperties.getFirstResolution());
    }
}