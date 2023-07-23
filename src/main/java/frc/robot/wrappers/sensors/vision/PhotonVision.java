package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;
import org.littletonrobotics.junction.Logger;

public class PhotonVision extends SubsystemBase {
    private final PhotonVisionIO photonVisionIO;
    private final PhotonVisionIOInputsAutoLogged inputs;
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;

    public PhotonVision(
            final PhotonVisionIO photonVisionIO,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator,
            final Field2d field2d
    ) {
        this.photonVisionIO = photonVisionIO;
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.field2d = field2d;

        this.inputs = new PhotonVisionIOInputsAutoLogged();
    }

    public PhotonVisionIO getPhotonVisionIO() {
        return photonVisionIO;
    }

    @Override
    public void periodic() {
        photonVisionIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        photonVisionIO.periodic();

        poseEstimator.update(
                swerve.getYaw(),
                swerve.getModulePositions()
        );

        final Pose2d estimatedPosition = poseEstimator.getEstimatedPosition();

        field2d.setRobotPose(estimatedPosition);
        Logger.getInstance().recordOutput("Odometry/Robot", estimatedPosition);
    }
}
