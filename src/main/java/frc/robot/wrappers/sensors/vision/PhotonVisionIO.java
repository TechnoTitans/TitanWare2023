package frc.robot.wrappers.sensors.vision;

import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

public interface PhotonVisionIO {
    class PhotonVisionIOInputs implements LoggableInputs {
        public double[] pipelineResultTargets = new double[] {};
        public String primaryStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR.toString();
        public EstimatedRobotPose estimatedRobotPose;
        public EstimatedRobotPose stableEstimatedRobotPose;

        @Override
        public void toLog(final LogTable table) {
            table.put("PipelineResultTargets", pipelineResultTargets);
            table.put("PrimaryStrategy", primaryStrategy);

            LogUtils.serializePhotonVisionEstimatedRobotPose(
                    table, "EstimatedRobotPose", estimatedRobotPose
            );
            LogUtils.serializePhotonVisionEstimatedRobotPose(
                    table, "StableEstimatedRobotPose", stableEstimatedRobotPose
            );
        }

        @Override
        public void fromLog(LogTable table) {
            this.pipelineResultTargets = table.getDoubleArray("PipelineResultTargets", new double[] {});
            this.primaryStrategy = table.getString(
                    "PrimaryStrategy", PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR.toString()
            );

            this.estimatedRobotPose =
                    LogUtils.deserializePhotonVisionEstimatedRobotPose(table, "EstimatedRobotPose");
            this.stableEstimatedRobotPose =
                    LogUtils.deserializePhotonVisionEstimatedRobotPose(table, "StableEstimatedRobotPose");
        }
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see PhotonVisionIOInputs
     */
    default void updateInputs(final PhotonVisionIOInputs inputs) {}

    default void periodic() {}
}
