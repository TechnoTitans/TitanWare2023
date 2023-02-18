package frc.robot;

import edu.wpi.first.wpilibj.I2C;

@SuppressWarnings("unused")
public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 1;

    //Sensors
    int PIGEON_ID = 1;
    I2C.Port CLAW_COLOR_SENSOR = I2C.Port.kOnboard;

    //Vision
    String PhotonVision_AprilTag_Cam = "apriltag1";

    //Canivore
    int CANIVORE_ID = 1;
    String CANIVORE_CAN_NAME = "CANivore";

    //LEDS
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
    int leftVerticalFalcon = 10;
    boolean leftElevatorMotorR = true;

    int horizontalElevatorNeo = 2;

    int clawTiltNeo = 3;

    //Claw Motors
    int clawOpenCloseMotor = 6;
    boolean clawOpenCloseMotorR = true;
    int clawOpenCloseEncoder = 9;

    int clawMainWheelsMotor = 7;
    boolean clawMainWheelsMotorR = false;
    int clawFollowerWheelsMotor = 8;
    boolean clawFollowerWheelsMotorR = true;

    //Controllers
    int MainController = 0;
    int CoController = 1;
}