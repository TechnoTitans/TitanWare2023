package frc.robot.wrappers.sensors.vision.replacements;

import frc.robot.utils.vision.TitanCamera;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.SimPhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.common.networktables.NTTopicSet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.lang.reflect.Field;
import java.util.List;

public class SimTitanCamera extends SimPhotonCamera {
    private final TitanCamera titanCamera;

    private final NTTopicSet ntTopicSet;
    private PhotonPipelineResult latestResult;
    private long heartbeatCounter = 0;

    public SimTitanCamera(final TitanCamera titanCamera) {
        super(titanCamera.getPhotonCamera().getName());

        this.titanCamera = titanCamera;
        setCameraIntrinsicsMat(titanCamera.getInstrinsicsMatrix());
        setCameraDistortionMat(titanCamera.getDistortionMatrix());

        try {
            final Field photonNTTopicSetField = this.getClass().getSuperclass().getDeclaredField("ts");
            photonNTTopicSetField.setAccessible(true);

            this.ntTopicSet = (NTTopicSet) photonNTTopicSetField.get(this);
        } catch (final NoSuchFieldException | IllegalAccessException reflectiveOperationException) {
            // no need to do anything special here, we are already in sim, so we don't really care
            // just go ahead and rethrow it
            throw new RuntimeException(reflectiveOperationException);
        }
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param sortMode Order in which to sort targets
     * @param targetList List of targets detected
     */
    public void submitProcessedFrame(
            double latencyMillis, PhotonTargetSortMode sortMode, List<PhotonTrackedTarget> targetList) {
        ntTopicSet.latencyMillisEntry.set(latencyMillis);

        Logger.getInstance().recordOutput(
                "PhotonResult_" + titanCamera.getPhotonCamera().getName() + "/Targets",
                targetList.stream().mapToDouble(PhotonTrackedTarget::getFiducialId).toArray()
        );

        if (sortMode != null) {
            targetList.sort(sortMode.getComparator());
        }

        final PhotonPipelineResult newResult = new PhotonPipelineResult(latencyMillis, targetList);
        final Packet newPacket = new Packet(newResult.getPacketSize());

        newResult.populatePacket(newPacket);

        ntTopicSet.rawBytesEntry.set(newPacket.getData());

        boolean hasTargets = newResult.hasTargets();
        ntTopicSet.hasTargetEntry.set(hasTargets);

//        if (!hasTargets) {
//            ts.targetPitchEntry.set(0.0);
//            ts.targetYawEntry.set(0.0);
//            ts.targetAreaEntry.set(0.0);
//            ts.targetPoseEntry.set(new double[] {0.0, 0.0, 0.0});
//            ts.targetSkewEntry.set(0.0);
//        } else {
//            var bestTarget = newResult.getBestTarget();
//
//            ts.targetPitchEntry.set(bestTarget.getPitch());
//            ts.targetYawEntry.set(bestTarget.getYaw());
//            ts.targetAreaEntry.set(bestTarget.getArea());
//            ts.targetSkewEntry.set(bestTarget.getSkew());
//
//            var transform = bestTarget.getBestCameraToTarget();
//            double[] poseData = {
//                    transform.getX(), transform.getY(), transform.getRotation().toRotation2d().getDegrees()
//            };
//            ts.targetPoseEntry.set(poseData);
//        }

        ntTopicSet.heartbeatPublisher.set(heartbeatCounter++);
        latestResult = newResult;
    }
}
