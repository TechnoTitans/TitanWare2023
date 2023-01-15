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
        Claw_RETRACTED, //Claw tilted up and closed
        CLAW_CLOSED, //Claw tilted down and closed
        CLAW_OPEN_SPINNING, //Claw tilted down and quickly spinning open
        CLAW_OPEN_STANDBY //Claw tilted down and slowly spinning open
    }

    public enum LimeLightLEDState {
        LED_OFF, //Limelight LEDS off
        LED_CONFIG, //Limelight LEDS to how they were set in the config
        LED_ON //Limelight LEDS on
    }
}
