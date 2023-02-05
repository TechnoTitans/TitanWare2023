package frc.robot;

@SuppressWarnings("unused")
public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 0;

    //Sensors
    int PIGEON_ID = 0;

    //Vision
    String PhotonVision_AprilTag_Cam = "apriltag1";

    //Canivore
    int CANIVORE_ID = 1;
    String CANIVORE_CAN_NAME = "CANivore";

    int CANdle_ID = 0;

    //Motors
        //Swerve
    int frontLeftDrive = 1;
    int frontLeftTurn = 2;
    int frontLeftEncoder = 1;
    boolean frontLeftDriveR = true;
    boolean frontLeftTurnR = false;

    int frontRightDrive = 3;
    int frontRightTurn = 4;
    int frontRightEncoder = 2;
    boolean frontRightDriveR = false;
    boolean frontRightTurnR = false;

    int backRightDrive = 5;
    int backRightTurn = 6;
    int backRightEncoder = 3;
    boolean backRightDriveR = false;
    boolean backRightTurnR = false;

    int backLeftDrive = 7;
    int backLeftTurn = 8;
    int backLeftEncoder = 4;
    boolean backLeftDriveR = true;
    boolean backLeftTurnR = false;

        //Elevator
    int leftVerticalFalcon = 5;
    boolean leftElevatorMotorR = false;

    int horizontalElevatorNeo = 3;

    int clawTiltNeo = 21;

    //Claw Motors
    int clawOpenCloseMotor = 12;
    boolean clawOpenCloseMotorR = true;

    int clawMainWheelsMotor = 15;
    boolean clawMainWheelsMotorR = false;
    int clawFollowerWheelsMotor = 13;
    boolean clawFollowerWheelsMotorR = true;

    //Controllers
    int MainController = 1;
    int CoController = 1;
}