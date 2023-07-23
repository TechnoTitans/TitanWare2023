package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

@SuppressWarnings("unused")
public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 1;

    //Sensors
    int PIGEON_ID = 1;

    //Vision
    String PhotonVision_Driver_Cam = "drivercam1";
    String PhotonVision_FR_Apriltag_R = "FR_Apriltag_R";
    String PhotonVision_FR_Apriltag_F = "FR_Apriltag_F";

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
    InvertedValue frontLeftDriveR = InvertedValue.CounterClockwise_Positive;
    InvertedValue frontLeftTurnR = InvertedValue.Clockwise_Positive;

    int frontRightDrive = 3;
    int frontRightTurn = 4;
    int frontRightEncoder = 2;
    InvertedValue frontRightDriveR = InvertedValue.CounterClockwise_Positive;
    InvertedValue frontRightTurnR = InvertedValue.Clockwise_Positive;

    int backRightDrive = 5;
    int backRightTurn = 6;
    int backRightEncoder = 3;
    InvertedValue backRightDriveR = InvertedValue.CounterClockwise_Positive;
    InvertedValue backRightTurnR = InvertedValue.Clockwise_Positive;

    int backLeftDrive = 7;
    int backLeftTurn = 8;
    int backLeftEncoder = 4;
    InvertedValue backLeftDriveR = InvertedValue.CounterClockwise_Positive;
    InvertedValue backLeftTurnR = InvertedValue.Clockwise_Positive;

    //Elevator
    int mainVerticalFalcon = 10;
    InvertedValue mainVerticalFalconR = InvertedValue.Clockwise_Positive;
    int followerVerticalFalcon = 11;
    InvertedValue followerVerticalFalconR = InvertedValue.Clockwise_Positive;
    int verticalElevatorEncoder = 12;
    int horizontalElevatorEncoder = 13;
    SensorDirectionValue verticalElevatorEncoderR = SensorDirectionValue.CounterClockwise_Positive;
    int horizontalElevatorNeo = 2;
    int verticalLimitSwitch = 6;
    int horizontalLimitSwitch = 8;
    int horizontalLimitHighSwitch = 5;

    //Claw Motors
    int clawTiltNeo = 3;
    int clawOpenCloseMotor = 6;
    boolean clawOpenCloseMotorR = false;
    int clawOpenCloseEncoder = 9;
    int clawTiltEncoder = 14;
    int clawLimitSwitch = 7;

    int clawMainWheelsMotor = 7;
    boolean clawMainWheelsMotorR = false;
    int clawFollowerWheelsMotor = 8;
    boolean clawFollowerWheelsMotorR = true;

    //Controllers
    int MainController = 0;
    int CoController = 1;
}