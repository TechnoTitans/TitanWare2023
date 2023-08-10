package frc.robot.wrappers.leds;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Enums;

public class CandleController {
    private final CANdle candle;
    private Enums.CANdleState currentState;

    public CandleController(final CANdle candle) {
        this.candle = candle;
    }

    public Enums.CANdleState getCurrentState() {
        return currentState;
    }

    public void setState(final Enums.CANdleState newState) {
        this.candle.setLEDs(newState.getR(), newState.getG(), newState.getB());
        this.currentState = newState;
    }

    public void setStrobe(final Enums.CANdleState newState, final double strobeTime) {
//        this.candle.animate(
//                new StrobeAnimation(
//                        state.getR(), state.getG(), state.getB(), 0, 0.5, -1
//                )
//        );

        //todo: apparenly the thing above was throwing an error but I cant replicate it so I made the thing on
        // the bottom just in case

        Commands.repeatingSequence(
                Commands.runOnce(() -> setState(newState)),
                Commands.waitSeconds(strobeTime),
                Commands.runOnce(() -> setState(Enums.CANdleState.OFF)),
                Commands.waitSeconds(strobeTime)
        );
    }
}
