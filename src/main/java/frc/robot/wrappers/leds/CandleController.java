package frc.robot.wrappers.leds;

import com.ctre.phoenix.led.CANdle;
import frc.robot.utils.Enums;

public class CandleController {
    private final CANdle caNdle;
    private Enums.CANdleState state;

    public CandleController(CANdle caNdle) {
        this.caNdle = caNdle;

        caNdle.configFactoryDefault();
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

    public Enums.CANdleState getState() {
        return state;
    }
}
