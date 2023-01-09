package frc.robot;

@SuppressWarnings("unused")
public interface RobotMap {
    //Compressor
    int PNEUMATICS_HUB_ID = 15;

    //Sensors
    int PIGEON_ID = 0;

    //Solenoids
    int GEAR_SHIFT_SOLENOID = 1;

    //Motors
    int LEFT_TALON_FRONT = 3, LEFT_TALON_BACK = 1;
    boolean REVERSED_LF_TALON = false, REVERSED_LB_TALON = false;

    int RIGHT_TALON_FRONT = 2, RIGHT_TALON_BACK = 0;
    boolean REVERSED_RF_TALON = true, REVERSED_RB_TALON = true;

    int LEFT_TALON_CLIMB = 10, RIGHT_TALON_CLIMB = 11;
    boolean REVERSED_LEFT_TALON = false, REVERSED_RIGHT_TALON = true;

    int MainController = 2;
    int CoController = 1;
}
