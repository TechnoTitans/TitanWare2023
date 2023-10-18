package frc.robot.utils.logging;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import org.littletonrobotics.junction.LogTable;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class LogUtils {
    public static final double MICRO_TO_MILLI = 1d / 1000;
    public static double microsecondsToMilliseconds(final double microseconds) {
        return microseconds * MICRO_TO_MILLI;
    }

    public static double[] toDoubleArray(final ChassisSpeeds chassisSpeeds) {
        return new double[] {
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond
        };
    }

    public static void serializePhotonVisionEstimatedRobotPose(
            final LogTable logTable,
            final String prefix,
            final EstimatedRobotPose estimatedRobotPose
    ) {
        if (estimatedRobotPose == null) {
            logTable.put(prefix + "_IsNull", true);
            return;
        } else {
            logTable.put(prefix + "_IsNull", false);
        }

        LogUtils.serializePose3d(logTable,  prefix + "_EstimatedPose", estimatedRobotPose.estimatedPose);
        logTable.put(prefix + "_TimestampSeconds", estimatedRobotPose.timestampSeconds);

        final int targetsUsedSize = estimatedRobotPose.targetsUsed.size();
        final int packetSize = targetsUsedSize * PhotonTrackedTarget.PACK_SIZE_BYTES;
        for (int i = 0; i < targetsUsedSize; i++) {
            final PhotonTrackedTarget trackedTarget = estimatedRobotPose.targetsUsed.get(i);
            final Packet packet = trackedTarget.populatePacket(new Packet(packetSize));

            LogUtils.serializePhotonVisionPacket(logTable, "_TargetsUsed_Packet_" + i, packet);
        }

        logTable.put(prefix + "_TargetsUsed_Size", targetsUsedSize);
    }

    public static EstimatedRobotPose deserializePhotonVisionEstimatedRobotPose(
            final LogTable logTable,
            final String prefix
    ) {
        if (logTable.getBoolean(prefix + "_IsNull", true)) {
            return null;
        }

        final Pose3d estimatedPose = LogUtils.deserializePose3d(logTable, "EstimatedRobotPose_EstimatedPose");
        final double timestampSeconds = logTable.getDouble("EstimatedRobotPose_TimestampSeconds", -1);

        final long targetsUsedSize = logTable.getInteger(prefix + "_TargetsUsed_Size", 0);
        final List<PhotonTrackedTarget> trackedTargets = new ArrayList<>();

        for (int i = 0; i < targetsUsedSize; i++) {
            final Packet packet = LogUtils.deserializePhotonVisionPacket(
                    logTable, "EstimatedRobotPose_TargetsUsed_Packet_" + i
            );

            final PhotonTrackedTarget trackedTarget = new PhotonTrackedTarget();
            trackedTarget.createFromPacket(packet);
            trackedTargets.add(trackedTarget);
        }

        return new EstimatedRobotPose(
                estimatedPose,
                timestampSeconds,
                trackedTargets
        );
    }

    public static void serializePhotonVisionPacket(final LogTable logTable, final String prefix, final Packet packet) {
        logTable.put(prefix + "_packetData", packet.getData());
    }

    public static Packet deserializePhotonVisionPacket(final LogTable logTable, final String prefix) {
        return new Packet(logTable.getRaw(prefix + "_packetData", new byte[] {}));
    }

    public static void serializePose3d(final LogTable logTable, final String prefix, final Pose3d pose3d) {
        logTable.put(prefix + "_Pose3d_x", pose3d.getX());
        logTable.put(prefix + "_Pose3d_y", pose3d.getY());
        logTable.put(prefix + "_Pose3d_z", pose3d.getZ());
        logTable.put(prefix + "_Pose3d_rw", pose3d.getRotation().getQuaternion().getW());
        logTable.put(prefix + "_Pose3d_rx", pose3d.getRotation().getQuaternion().getX());
        logTable.put(prefix + "_Pose3d_ry", pose3d.getRotation().getQuaternion().getY());
        logTable.put(prefix + "_Pose3d_rz", pose3d.getRotation().getQuaternion().getZ());
    }

    public static Pose3d deserializePose3d(final LogTable logTable, final String prefix) {
        return new Pose3d(
                logTable.getDouble(prefix + "_Pose3d_x", 0),
                logTable.getDouble(prefix + "_Pose3d_y", 0),
                logTable.getDouble(prefix + "_Pose3d_z", 0),
                new Rotation3d(new Quaternion(
                        logTable.getDouble(prefix + "_Pose3d_rw", 1),
                        logTable.getDouble(prefix + "_Pose3d_rx", 0),
                        logTable.getDouble(prefix + "_Pose3d_ry", 0),
                        logTable.getDouble(prefix + "_Pose3d_rz", 0)
                ))
        );
    }

    /**
     * Since PathPlanner trajectories seem to have >1k (even up to 2k) states in each trajectory,
     * AdvantageScope struggles to log all the states (poses) and it lags out the interface
     * <p>
     * This class reduces the number of states down to a maximum of {@link LoggableTrajectory#MAX_STATES} so that
     * it can be properly logged
     * <p>
     * Do <b>NOT</b> rely on this class to provide accurate states - this should ONLY be used for logging purposes
     * @see org.littletonrobotics.junction.Logger#recordOutput(String, Trajectory)
     * @see Trajectory
     */
    public static class LoggableTrajectory extends Trajectory {
        public static final double TIME_ACCURACY = 0.1;
        public static final int MAX_STATES = 25;

        public LoggableTrajectory(final List<State> states) {
            super(states);
        }

        /**
         * Create a new {@link LoggableTrajectory} from a {@link Trajectory} object.
         * @param trajectory the {@link Trajectory} to create from
         * @return the new {@link LoggableTrajectory}
         * @see Trajectory
         */
        public static LoggableTrajectory fromTrajectory(final Trajectory trajectory) {
            return new LoggableTrajectory(trajectory.getStates());
        }

        /**
         * Return a reduced number of states (a maximum of {@link LoggableTrajectory#MAX_STATES})
         * in this {@link LoggableTrajectory}.
         * <p>
         * To get the entire {@link List} of {@link State}s,
         * use {@link LoggableTrajectory#getAllStates()}
         * <p>
         * Do <b>NOT</b> rely on this to report an accurate list of states
         * @return the reduced {@link List} of {@link State}s
         * @see Trajectory#getStates()
         */
        @Override
        public List<State> getStates() {
            final double totalTime = this.getTotalTimeSeconds();
            final double timeDiffPerReportedState = Math.max(totalTime / MAX_STATES, TIME_ACCURACY);

            final List<State> allStates = super.getStates();
            final List<State> reportedStates = new ArrayList<>(Math.min(allStates.size(), MAX_STATES));

            // return empty list when there are no states, fast-path if there's only one state
            if (allStates.isEmpty()) {
                return List.of();
            } else if (allStates.size() == 1) {
                return List.of(allStates.get(0));
            }

            State lastState = allStates.get(0);
            for (double t = allStates.get(1).timeSeconds; t <= totalTime; t += timeDiffPerReportedState) {
                final State nextState = this.sample(t);
                if (nextState != lastState) {
                    reportedStates.add(nextState);
                    lastState = nextState;
                }
            }

            return reportedStates;
        }

        /**
         * Returns all states without any reduction. Equivalent to {@link Trajectory#getStates()}
         * @return the list of states
         */
        public List<State> getAllStates() {
            return super.getStates();
        }
    }
}
