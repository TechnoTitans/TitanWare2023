package frc.robot.utils.sim;

import edu.wpi.first.wpilibj.DigitalInput;

@SuppressWarnings("unused")
public class LimitSwitchSim {
    private final int channel;
    private boolean initialized;
    private boolean value;

    public LimitSwitchSim(final int channel) {
        this.channel = channel;
        resetData();
    }

    public LimitSwitchSim(final DigitalInput digitalInput) {
        this(digitalInput.getChannel());
    }

    /**
     * Check whether this DIO has been initialized.
     *
     * @return true if initialized
     */
    public boolean getInitialized() {
        return initialized;
    }

    /**
     * Define whether this DIO has been initialized.
     *
     * @param initialized whether this object is initialized
     */
    public void setInitialized(boolean initialized) {
        this.initialized = initialized;
    }

    /**
     * Read the value of the DIO port.
     *
     * @return the DIO value
     */
    public boolean getValue() {
        return value;
    }

    /**
     * Change the DIO value.
     *
     * @param value the new value
     */
    public void setValue(boolean value) {
        if (this.value != value) {
            this.value = value;
        }
    }

    /**
     * Get the channel of this LimitSwitchSim.
     * @return the channel
     */
    public int getChannel() {
        return channel;
    }

    /** Reset all simulation data of this object. */
    public void resetData() {
        this.initialized = false;
        this.value = false;
    }
}
