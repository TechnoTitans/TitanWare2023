package frc.robot;

@SuppressWarnings("unused")
public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 0;

    //Compressor
    int PNEUMATICS_HUB_ID = 15;

    //Sensors
    int PIGEON_ID = 0;

    //Canivore
    int CANIVORE_ID = 1;
    String CANIVORE_CAN_NAME = "canivore";

    //Motors
    int frontLeftDrive = 12;
    int frontLeftTurn = 14;
    int frontLeftEncoder = 1;
    boolean frontLeftDriveR = false;
    boolean frontLeftTurnR = false;

    int frontRightDrive = 5;
    int frontRightTurn = 4;
    int frontRightEncoder = 0;
    boolean frontRightDriveR = false;
    boolean frontRightTurnR = false;

    int backLeftDrive = 8;
    int backLeftTurn = 6;
    int backLeftEncoder = 4;
    boolean backLeftDriveR = false;
    boolean backLeftTurnR = false;

    int backRightDrive = 2;
    int backRightTurn = 5;
    int backRightEncoder = 3;
    boolean backRightDriveR = false;
    boolean backRightTurnR = false;

    int MainController = 2;
    int CoController = 1;
}