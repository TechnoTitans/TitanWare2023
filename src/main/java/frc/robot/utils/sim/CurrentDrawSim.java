package frc.robot.utils.sim;

import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;

import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class CurrentDrawSim extends VirtualSubsystem {
    private static final int RESERVED_CHANNEL = -1;
    private static CurrentDrawSim INSTANCE;

    public static CurrentDrawSim getInstance() {
        if (INSTANCE == null) {
            return (INSTANCE = new CurrentDrawSim());
        }

        return INSTANCE;
    }

    private final LoggedPowerDistribution powerDistribution = LoggedPowerDistribution.getInstance();
    private final HashMap<Integer, Double> currentDraws = new HashMap<>(Map.of(RESERVED_CHANNEL, 0d));
    private boolean isDataOld = false;

    public void report(final double currentDrawAmps, final int... channels) {
        for (final int channel : channels) {
            if (channel == RESERVED_CHANNEL) {
                throw new IllegalArgumentException("Attempted to report current to RESERVED_CHANNEL!");
            }

            final LoggedPowerDistribution.PowerDistributionInputs inputs = powerDistribution.getInputs();
            final double adjustedCurrentDrawAmps;
            if (channel >= 1 && channel <= inputs.channelCount && channel <= inputs.pdpChannelCurrents.length) {
                adjustedCurrentDrawAmps = currentDrawAmps + inputs.pdpChannelCurrents[channel - 1];
            } else {
                adjustedCurrentDrawAmps = currentDrawAmps;
            }

            if (currentDraws.put(channel, adjustedCurrentDrawAmps) != null && !isDataOld) {
                throw new RuntimeException("Attempted to report duplicate channel within 1 loop period!");
            }
        }

        if (isDataOld && channels.length > 0) {
            isDataOld = false;
        }
    }

    public void report(final double currentDrawAmps) {
        if (isDataOld) {
            this.isDataOld = false;
        }

        currentDraws.merge(RESERVED_CHANNEL, currentDrawAmps, Double::sum);
    }

    public double getTotalCurrentDraw() {
        return currentDraws.values().stream()
                .mapToDouble(Double::doubleValue)
                .sum();
    }

    @Override
    public void periodic() {
        this.isDataOld = true;
    }
}
