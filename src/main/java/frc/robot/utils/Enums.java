package frc.robot.utils;

public class Enums {
    public enum ElevatorState {
        ELEVATOR_EXTENDED_HIGH, //Elevator High and Horizontal extended
        ELEVATOR_EXTENDED_MID, //Elevator Mid and Horizontal extended
        ELEVATOR_EXTENDED_PLATFORM, //Elevator Platform and Horizontal extended
        ELEVATOR_STANDBY, //Elevator at normal height
        ELEVATOR_CUBE,
        SINGLE_SUB
    }

    public enum ClawState {
        CLAW_OUTTAKE, //Claw shoot cube
        CLAW_HOLDING, //Claw tilted down and closed
        CLAW_INTAKING_CONE, //Claw tilted down and open cone
        CLAW_INTAKING_CUBE, //Claw tilted down and open cube
        CLAW_STANDBY, //Claw tilted down and standby wheel speed
        CLAW_DROP, //Drop claw to outtake height
        CLAW_ANGLE_SHOOT,
        CLAW_SHOOT_HIGH,
        CLAW_SHOOT_LOW,
        CLAW_ANGLE_CUBE,
        SINGLE_SUB
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

    public enum SwerveSpeeds {
        FAST,
        NORMAL,
        SLOW,
    }

    public enum GridPositions {
        LEFT,
        CENTER,
        RIGHT,
    }
}
