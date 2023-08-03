package frc.robot.wrappers.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.utils.Enums;

public class CandleController {
    private final CANdle caNdle;
    private Enums.CANdleState state;

    public CandleController(final CANdle caNdle) {
        this.caNdle = caNdle;
    }

    public Enums.CANdleState getState() {
        return state;
    }

    public void setState(final Enums.CANdleState state) {
        this.caNdle.setLEDs(state.getR(), state.getG(), state.getB());
        this.state = state;
    }

    public void setStrobe(final Enums.CANdleState state) {
        this.caNdle.animate(
                new StrobeAnimation(
                        state.getR(), state.getG(), state.getB(), 0, 0.5, -1
                )
        );
    }
}
