package frc.robot.utils.logging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

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
