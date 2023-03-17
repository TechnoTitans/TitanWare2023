package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

@SuppressWarnings("unused")
public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 1;

    //Sensors
    int PIGEON_ID = 1;
//    I2C.Port CLAW_COLOR_SENSOR = I2C.Port.kOnboard;

    //Vision
    String PhotonVision_Driver_Cam = "drivercam1";
    String PhotonVision_AprilTag_Cam = "apriltag1";
    Transform3d robotToCam = new Transform3d(
                    new Translation3d(0.5, 0.0, 0.5),
                    new Rotation3d(0, 0, 0));
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.

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
    int mainVerticalFalcon = 10;
    boolean mainVerticalFalconR = true;
    int horizontalElevatorNeo = 2;
    int verticalLimitSwitch = 6;
    int horizontalLimitSwitch = 8;

    //Claw Motors
    int clawTiltNeo = 3;
    int clawOpenCloseMotor = 6;
    boolean clawOpenCloseMotorR = false;
    int clawOpenCloseEncoder = 9;
    int clawLimitSwitch = 7;

    int clawMainWheelsMotor = 7;
    boolean clawMainWheelsMotorR = false;
    int clawFollowerWheelsMotor = 8;
    boolean clawFollowerWheelsMotorR = true;

    //Controllers
    int MainController = 0;
    int CoController = 1;
}