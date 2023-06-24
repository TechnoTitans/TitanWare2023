package frc.robot.wrappers.leds;

import com.ctre.phoenix.led.CANdle;
import frc.robot.utils.Enums;

@SuppressWarnings("unused")
public class CandleController {
    private final CANdle caNdle;
    private Enums.CANdleState state;

    public CandleController(final CANdle caNdle) {
        this.caNdle = caNdle;
    }

    public Enums.CANdleState getState() {
        return state;
    }

    public void setState(Enums.CANdleState state) {
        this.state = state;
        switch (state) {
            case OFF:
                caNdle.setLEDs(0, 0, 0);
                break;
            case YELLOW:
                caNdle.setLEDs(200, 100, 0);
                break;
            case PURPLE:
                caNdle.setLEDs(200, 0, 150);
                break;
        }
    }
}
