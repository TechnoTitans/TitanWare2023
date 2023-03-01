package frc.robot.utils;

public class Enums {
    public enum ElevatorState {
        ELEVATOR_EXTENDED_HIGH, //Elevator High and Horizontal extended
        ELEVATOR_EXTENDED_MID, //Elevator Mid and Horizontal extended
        ELEVATOR_EXTENDED_PLATFORM, //Elevator Platform and Horizontal extended
        ELEVATOR_STANDBY, //Elevator at normal height
    }

    public enum ClawState {
        CLAW_OUTTAKE, //Claw shoot cube
        CLAW_HOLDING, //Claw tilted down and closed
        CLAW_INTAKEING, //Claw tilted down and open
        CLAW_STANDBY, //Claw tilted down and standby wheel speed
        CLAW_DROP //Drop claw to outtake height
    }

    public enum CANdleState {
        OFF,
        YELLOW,
        PURPLE
    }

    public enum DriverProfiles {
        DRIVER1,
        DRIVER2
    }

    public enum LimeLightLEDState {
        LED_OFF, //Limelight LEDS off
        LED_CONFIG, //Limelight LEDS to how they were set in the config
        LED_ON //Limelight LEDS on
    }

    public enum VisionMode {
        PHOTON_VISION,
        LIME_LIGHT
    }
}
