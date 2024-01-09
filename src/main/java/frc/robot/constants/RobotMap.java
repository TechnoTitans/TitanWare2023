package frc.robot.constants;

public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 0;

    //Sensors
    int PIGEON_ID = 13;

    //Vision
    String photonvision_driver_cam = "drivercam1";
    String photonvision_FR_apriltag_R = "FR_Apriltag_R";
    String photonvision_FR_apriltag_F = "FR_Apriltag_F";
    String photonvision_FL_apriltag_L = "FL_Apriltag_L";
    String photonvision_BR_apriltag_B = "BR_Apriltag_B";

    //CAN Bus
    String rioCANBus = "rio";
    String canivoreCANBus = "CANivore";

    //LEDs
    int candle = 25;

    //Controllers
    int MainController = 0;
    int CoController = 1;
}