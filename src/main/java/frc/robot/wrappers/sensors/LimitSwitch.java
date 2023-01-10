package frc.robot.wrappers.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

@SuppressWarnings("unused")
public class LimitSwitch extends DigitalInput {
    private final boolean inverted;

    public LimitSwitch(int channel, boolean inverted) {
        super(channel);
        this.inverted = inverted;
    }

    public boolean isPressed() {
        return this.get() == !inverted;
    }
}
