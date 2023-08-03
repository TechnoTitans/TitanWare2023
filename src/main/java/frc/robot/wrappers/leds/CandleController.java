package frc.robot.wrappers.leds;

import com.ctre.phoenix.led.CANdle;
import frc.robot.utils.Enums;

public class CandleController {
    private final CANdle candle;
    private Enums.CANdleState state;

    public CandleController(final CANdle candle) {
        this.candle = candle;
    }

    public Enums.CANdleState getState() {
        return state;
    }

    public void setState(final Enums.CANdleState state) {
        this.candle.setLEDs(state.getR(), state.getG(), state.getB());
        this.state = state;
    }

    public void setStrobe(final Enums.CANdleState state) {
        //TODO: fix this throwing some error
//        this.candle.animate(
//                new StrobeAnimation(
//                        state.getR(), state.getG(), state.getB(), 0, 0.5, -1
//                )
//        );
    }
}
