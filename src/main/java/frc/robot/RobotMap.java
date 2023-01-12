package frc.robot;

@SuppressWarnings("unused")
public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 0;

    //Compressor
    int PNEUMATICS_HUB_ID = 15;

        //Solenoids
        int CLAW_SOLENOID = 1;

    //Sensors
    int PIGEON_ID = 0;

    //Canivore
    int CANIVORE_ID = 1;
    String CANIVORE_CAN_NAME = "canivore";

    //Motors
        //Swerve
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

        //Elevator
    int leftVerticalFalcon = 5;
    boolean leftElevatorMotorR = false;
    int rightVerticalFalcon = 6;
    boolean rightElevatorMotorR = false;
    int verticalElevatorSRXMAG = 3;

    int horizontalElevatorNeo = 3;
    int horizontalElevatorSRXMAG = 3;

    int clawTiltBag = 3;
    boolean clawTiltBagR = false;
    int clawTiltElevator550SRXMAG = 3;

    int clawWheels550 = 3;

    int MainController = 2;
    int CoController = 1;
}