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
    int rightVerticalFalcon = 6;
    boolean rightElevatorMotorR = false;
    int verticalElevatorSRXMAG = 3;

    int horizontalElevatorNeo = 3;
    int horizontalElevatorSRXMAG = 3;

    int clawTilt550 = 3;
    int clawTiltElevator550SRXMAG = 3;

    int clawWheelsLeftMotor = 3;
    boolean clawWheelsLeftMotorR = false;
    int clawWheelsRightMotor = 3;
    boolean clawWheelsRightMotorR = false;

    int MainController = 1;
    int CoController = 1;
}