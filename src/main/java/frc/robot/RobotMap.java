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
    int verticalElevatorCanCoder = 3;

    int horizontalElevatorNeo = 3;

    int clawTiltNeo = 21;

    int clawOpenCloseMotor = 3;
    boolean clawOpenCloseMotorR = false;

    int clawMainWheelsMotor = 3;
    boolean clawMainWheelsMotorR = false;
    int clawFollowerWheelsMotor = 3;
    boolean clawFollowerWheelsMotorR = false;

    //Controllers
    int MainController = 1;
    int CoController = 1;
}