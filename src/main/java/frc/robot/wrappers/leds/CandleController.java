package frc.robot.wrappers.leds;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotMap;
import frc.robot.utils.SuperstructureStates;

public class CandleController {
    private static CandleController INSTANCE;
    public static CandleController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CandleController(new CANdle(RobotMap.CANdle_ID));
        }
        return INSTANCE;
    }

    private final CANdle candle;
    private SuperstructureStates.CANdleState currentState;

    public CandleController(final CANdle candle) {
        this.candle = candle;
    }

    public SuperstructureStates.CANdleState getCurrentState() {
        return currentState;
    }

    public void setState(final SuperstructureStates.CANdleState newState) {
        this.candle.setLEDs(newState.getR(), newState.getG(), newState.getB());
        this.currentState = newState;
    }

    public void setStrobe(final SuperstructureStates.CANdleState newState, final double strobeTime) {
//        this.candle.animate(
//                new StrobeAnimation(
//                        state.getR(), state.getG(), state.getB(), 0, 0.5, -1
//                )
//        );

        //todo: apparently the thing above was throwing an error but I cant replicate it so I made the thing on
        // the bottom just in case

        Commands.repeatingSequence(
                Commands.runOnce(() -> setState(newState)),
                Commands.waitSeconds(strobeTime),
                Commands.runOnce(() -> setState(SuperstructureStates.CANdleState.OFF)),
                Commands.waitSeconds(strobeTime)
        );
    }
}
