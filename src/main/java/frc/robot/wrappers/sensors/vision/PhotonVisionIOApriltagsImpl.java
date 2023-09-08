package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.vision.TitanCamera;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import java.util.List;

public class PhotonVisionIOApriltagsImpl implements PhotonVisionIO {
    private final Swerve swerve;
    private final SwerveDriveOdometry visionIndependentOdometry;
    private final List<PhotonCamera> photonCameras;
    private final boolean canRun;

    private final boolean isReal;
    private final List<PhotonCameraSim> photonCameraSims;
    private final VisionSystemSim visionSystemSim;

    public PhotonVisionIOApriltagsImpl(
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final List<TitanCamera> apriltagCameras
    ) {
        this.swerve = swerve;
        this.visionIndependentOdometry = visionIndependentOdometry;
        this.photonCameras = apriltagCameras.stream().map(TitanCamera::getPhotonCamera).toList();
        this.isReal = Constants.CURRENT_MODE == Constants.RobotMode.REAL;

        final AprilTagFieldLayout apriltagFieldLayout = PhotonVision.apriltagFieldLayout;
        if (apriltagFieldLayout == null) {
            if (!isReal) {
                throw new RuntimeException("Attempted to run PhotonVision while the ApriltagFieldLayout is null!");
            } else {
                DriverStation.reportError(
                        "Ran PhotonVision while the ApriltagFieldLayout is null! PhotonVision will not work!",
                        true
                );
            }
        }

        this.canRun = apriltagFieldLayout != null;
        if (!isReal) {
            this.visionSystemSim = new VisionSystemSim(PhotonVision.photonLogKey);
            this.visionSystemSim.addVisionTargets(PhotonVision.apriltagFieldLayout);

            this.photonCameraSims = apriltagCameras
                    .stream()
                    .map(titanCamera -> {
                        final PhotonCameraSim photonCameraSim =
                                new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

                        visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotRelativeToCameraTransform());
                        ToClose.add(photonCameraSim);

                        return photonCameraSim;
                    })
                    .toList();
        } else {
            this.photonCameraSims = List.of();
            this.visionSystemSim = null;
        }
    }

    @Override
    public void periodic() {
        if (ToClose.hasClosed() || !canRun) {
            // do not try to update if we've already closed or if we cannot continue running
            return;
        }

        if (!isReal) {
            final Pose2d visionIndependentPose = visionIndependentOdometry.getPoseMeters();
            visionSystemSim.update(
                    GyroUtils.robotPose2dToPose3dWithGyro(
                            visionIndependentPose,
                            GyroUtils.rpyToRotation3d(swerve.getRoll(), swerve.getPitch(), swerve.getYaw())
                    )
            );
        }
    }

    @Override
    public void resetRobotPose(final Pose3d robotPose) {
        if (!isReal && canRun) {
            visionSystemSim.resetRobotPose(robotPose);
        }
    }
}
