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
import org.w3c.dom.css.Counter;

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
    Transform3d robotToCam = new Transform3d( //x, y, z
                    new Translation3d(Units.inchesToMeters(0.5), Units.inchesToMeters(-12.625), Units.inchesToMeters(25)),
                    new Rotation3d(0, 0, 0));
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    // Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
    // matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
    Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // Units.degreesToRadians(5)
    //Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    // less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9); // Units.degreesToRadians(10)


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
    SensorDirectionValue verticalElevatorEncoderR = SensorDirectionValue.Clockwise_Positive;
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