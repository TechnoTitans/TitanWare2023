package frc.robot;

import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

@SuppressWarnings("unused")
public interface RobotMap {
    //PDP
    int POWER_DISTRIBUTION_HUB = 1;

    //Sensors
    int PIGEON_ID = 1;

    //Vision
    String PhotonVision_Driver_Cam = "drivercam1";
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
    InvertedValue frontLeftDriveR = InvertedValue.Clockwise_Positive;
    boolean frontLeftTurnR = false;

    int frontRightDrive = 3;
    int frontRightTurn = 4;
    int frontRightEncoder = 2;
    InvertedValue frontRightDriveR = InvertedValue.CounterClockwise_Positive;
    boolean frontRightTurnR = false;

    int backRightDrive = 5;
    int backRightTurn = 6;
    int backRightEncoder = 3;
    InvertedValue backRightDriveR = InvertedValue.CounterClockwise_Positive;
    boolean backRightTurnR = false;

    int backLeftDrive = 7;
    int backLeftTurn = 8;
    int backLeftEncoder = 4;
    InvertedValue backLeftDriveR = InvertedValue.Clockwise_Positive;
    boolean backLeftTurnR = false;

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