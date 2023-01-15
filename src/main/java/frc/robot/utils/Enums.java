package frc.robot.utils;

public class Enums {
    public enum ElevatorState {
        ELEVATOR_EXTENDED_HIGH, //Elevator High and Dropper extended
        ELEVATOR_EXTENDED_MID, //Elevator Mid and Dropper extended
        ELEVATOR_EXTENDED_GROUND, //Elevator Low and Dropper extended
        ELEVATOR_EXTENDED_PLATFORM, //Elevator Low and Dropper extended
        ELEVATOR_STANDBY, //Elevator at pickup height and grabber open
        ELEVATOR_PREGAME //Elevator at pickup height and grabber tilted vertically
    }

    public enum ClawState {
        Claw_RETRACTED,
        CLAW_CLOSED,
        CLAW_OPEN_SPINNING,
        CLAW_OPEN_STANDBY
    }
}
