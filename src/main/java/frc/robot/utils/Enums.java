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
        CLAW_RETRACTED, //Claw tilted up and closed
        CLAW_OUTTAKE, //Claw shoot cube
        CLAW_HOLDING, //Claw tilted down and closed
        CLAW_DROP_CONE, //Claw tilted down and open
        CLAW_INTAKING, //Claw tilted down and intake wheel speed
        CLAW_STANDBY //Claw tilted down and standby wheel speed
    }

    public enum CANdleState {
        OFF,
        YELLOW,
        PURPLE
    }

    public enum LimeLightLEDSate {
        LED_OFF, //Limelight LEDS off
        LED_CONFIG, //Limelight LEDS to how they were set in the config
        LED_ON //Limelight LEDS on
    }

    public enum VisionMode {
        PHOTON_VISION,
        LIME_LIGHT
    }
}
