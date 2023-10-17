package frc.robot.utils.sim;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.Constants;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;

import java.util.HashMap;
import java.util.Map;

public class CurrentDrawSim extends VirtualSubsystem {
    protected static final String logKey = "CurrentDrawSim";
    private static final int RESERVED_CHANNEL = -1;
    private static CurrentDrawSim INSTANCE;

    /**
     * Whether the {@link CurrentDrawSim} is currently enabled/usable.
     * @return true if enabled/usable, false if not
     */
    public static boolean isEnabled() {
        return Constants.CURRENT_MODE == Constants.RobotMode.SIM;
    }

    /**
     * Get the {@link CurrentDrawSim} singleton, or create it and return it if it doesn't yet exist.
     * @return the {@link CurrentDrawSim} singleton
     */
    public static CurrentDrawSim getInstance() {
        if (!isEnabled()) {
            throw new RuntimeException("Attempted to use CurrentDrawSim when it is not enabled!");
        } else if (INSTANCE == null) {
           INSTANCE = new CurrentDrawSim();
        }

        return INSTANCE;
    }

    private final LoggedPowerDistribution powerDistribution;
    private final HashMap<Integer, Double> currentDraws;
    private boolean isDataOld = false;

    public CurrentDrawSim() {
        super();

        this.powerDistribution = LoggedPowerDistribution.getInstance();
        this.currentDraws = new HashMap<>(Map.of(RESERVED_CHANNEL, 0d));
    }

    private double getAdjustedCurrentDrawAmps(double currentDrawAmps, int channel) {
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

        return adjustedCurrentDrawAmps;
    }

    public void report(final double currentDrawAmps, final int... channels) {
        final int nChannels = channels.length;
        for (final int channel : channels) {
            final double averageCurrentDrawAmps = currentDrawAmps / nChannels;
            final double adjustedCurrentDrawAmps = getAdjustedCurrentDrawAmps(averageCurrentDrawAmps, channel);

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
        Logger.getInstance().recordOutput(logKey + "/TotalCurrentDraw", getTotalCurrentDraw());
        Logger.getInstance().recordOutput(logKey + "/CurrentDraws", currentDraws.values().stream()
                .mapToDouble(Double::doubleValue)
                .toArray()
        );

        RoboRioSim.setVInVoltage(
                Math.max(
                        BatterySim.calculateLoadedBatteryVoltage(
                                Constants.PDH.BATTERY_NOMINAL_VOLTAGE,
                                Constants.PDH.BATTERY_RESISTANCE_OHMS,
                                getTotalCurrentDraw()
                        ),
                        0
                )
        );
    }
}
