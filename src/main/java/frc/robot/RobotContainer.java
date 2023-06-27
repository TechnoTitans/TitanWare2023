package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.commands.teleop.AutoAlignment;
import frc.robot.commands.teleop.ElevatorTeleop;
import frc.robot.commands.teleop.IntakeTeleop;
import frc.robot.commands.teleop.SwerveDriveTeleop;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.DriveController;
import frc.robot.utils.Enums;
import frc.robot.wrappers.control.OI;
import frc.robot.wrappers.control.TitanButton;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;
import frc.robot.wrappers.sensors.vision.PhotonApriltags;
import frc.robot.wrappers.sensors.vision.PhotonDriverCam;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    //OI
    public final OI oi;

    //Motors
    public final TalonFX frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public final TalonFX frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    public final CANcoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    //Elevator
    public final TalonFX elevatorVerticalMotorMain, elevatorVerticalMotorFollower;
    public final CANcoder elevatorVerticalEncoder;
    public final CANCoder elevatorHorizontalEncoder;
    public final TitanMAX elevatorHorizontalNeo;
    public final DigitalInput elevatorVerticalLimitSwitch, elevatorHorizontalLimitSwitch, elevatorHorizontalHighLimitSwitch;

    //Claw
    public final TitanSRX clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor;
    public final CANCoder clawOpenCloseEncoder;
    public final CANCoder clawTiltEncoder;
    public final TitanMAX clawTiltNeo;
    public final DigitalInput clawTiltLimitSwitch;

    //PoseEstimation
    public final SwerveDrivePoseEstimator poseEstimator;

    //Swerve
    public final SwerveModule frontLeft, frontRight, backLeft, backRight;
    public final SwerveDriveKinematics kinematics;
    public final DriveController holonomicDriveController;
    public final Field2d field;

    //PDH
    public final PowerDistribution powerDistribution;

    //Sensors
    public final Pigeon2 pigeon;

    //Vision
    public final PhotonCamera photonDriveCamera, photonApriltagCamera;
    public final PhotonDriverCam photonDriverCam;
    public final PhotonApriltags photonApriltags;

    //Candle
    public final CANdle cANdle;
    public final CandleController candleController;

    //SubSystems
    public final Swerve swerve;
    public final Elevator elevator;
    public final Claw claw;

    //Teleop Commands
    public final SwerveDriveTeleop swerveDriveTeleop;
    public final AutoAlignment autoAlignment;
    public final IntakeTeleop intakeTeleop;
    public final ElevatorTeleop elevatorTeleop;

    //Buttons
    //Main Driver
    public final TitanButton resetGyroBtn, alignLeftBtn, alignRightBtn;
    //Co Driver
    public final TitanButton candleYellowBtn, candlePurpleBtn;

    //Autonomous Commands
    public final TrajectoryManager trajectoryManager;

    //SmartDashboard
    public final SendableChooser<Enums.DriverProfiles> profileChooser;
    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //OI
        oi = new OI();

        //Power Distribution Hub
        powerDistribution = new PowerDistribution(RobotMap.POWER_DISTRIBUTION_HUB, PowerDistribution.ModuleType.kRev);
        powerDistribution.clearStickyFaults();

        //Swerve Drive Motors
        frontLeftDrive = new TalonFX(RobotMap.frontLeftDrive, RobotMap.CANIVORE_CAN_NAME);
        frontRightDrive = new TalonFX(RobotMap.frontRightDrive, RobotMap.CANIVORE_CAN_NAME);
        backLeftDrive = new TalonFX(RobotMap.backLeftDrive, RobotMap.CANIVORE_CAN_NAME);
        backRightDrive = new TalonFX(RobotMap.backRightDrive, RobotMap.CANIVORE_CAN_NAME);

        //Swerve Turning Motors
        frontLeftTurn = new TalonFX(RobotMap.frontLeftTurn, RobotMap.CANIVORE_CAN_NAME);
        frontRightTurn = new TalonFX(RobotMap.frontRightTurn, RobotMap.CANIVORE_CAN_NAME);
        backLeftTurn = new TalonFX(RobotMap.backLeftTurn, RobotMap.CANIVORE_CAN_NAME);
        backRightTurn = new TalonFX(RobotMap.backRightTurn, RobotMap.CANIVORE_CAN_NAME);

        //Swerve CANCoders
        frontLeftEncoder = new CANcoder(RobotMap.frontLeftEncoder, RobotMap.CANIVORE_CAN_NAME);
        frontRightEncoder = new CANcoder(RobotMap.frontRightEncoder, RobotMap.CANIVORE_CAN_NAME);
        backLeftEncoder = new CANcoder(RobotMap.backLeftEncoder, RobotMap.CANIVORE_CAN_NAME);
        backRightEncoder = new CANcoder(RobotMap.backRightEncoder, RobotMap.CANIVORE_CAN_NAME);

        //Swerve Modules
        frontLeft = new SwerveModule(frontLeftDrive, frontLeftTurn, frontLeftEncoder, RobotMap.frontLeftDriveR, RobotMap.frontLeftTurnR, 0.322);
        frontRight = new SwerveModule(frontRightDrive, frontRightTurn, frontRightEncoder, RobotMap.frontRightDriveR, RobotMap.frontRightTurnR, -0.168);
        backLeft = new SwerveModule(backLeftDrive, backLeftTurn, backLeftEncoder, RobotMap.backLeftDriveR, RobotMap.backLeftTurnR, 0.05);
        backRight = new SwerveModule(backRightDrive, backRightTurn, backRightEncoder, RobotMap.backRightDriveR, RobotMap.backRightTurnR, -0.216);

        //Elevator Motors
        elevatorVerticalMotorMain = new TalonFX(RobotMap.mainVerticalFalcon, RobotMap.CANIVORE_CAN_NAME);
        elevatorVerticalMotorFollower = new TalonFX(RobotMap.followerVerticalFalcon, RobotMap.CANIVORE_CAN_NAME);
        elevatorVerticalEncoder = new CANcoder(RobotMap.verticalElevatorEncoder, RobotMap.CANIVORE_CAN_NAME);
        elevatorHorizontalEncoder = new CANCoder(RobotMap.horizontalElevatorEncoder);

        elevatorVerticalLimitSwitch = new DigitalInput(RobotMap.verticalLimitSwitch);
        elevatorHorizontalLimitSwitch = new DigitalInput(RobotMap.horizontalLimitSwitch);
        elevatorHorizontalHighLimitSwitch = new DigitalInput(RobotMap.horizontalLimitHighSwitch);

        clawMainWheelsMotor = new TitanSRX(RobotMap.clawMainWheelsMotor, RobotMap.clawMainWheelsMotorR);
        clawFollowerWheelsMotor = new TitanSRX(RobotMap.clawFollowerWheelsMotor, RobotMap.clawFollowerWheelsMotorR);
        clawOpenCloseMotor = new TitanSRX(RobotMap.clawOpenCloseMotor, RobotMap.clawOpenCloseMotorR);
        clawOpenCloseEncoder = new CANCoder(RobotMap.clawOpenCloseEncoder);

        elevatorHorizontalNeo = new TitanMAX(RobotMap.horizontalElevatorNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
        clawTiltNeo = new TitanMAX(RobotMap.clawTiltNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
        clawTiltEncoder = new CANCoder(RobotMap.clawTiltEncoder);
        clawTiltLimitSwitch = new DigitalInput(RobotMap.clawLimitSwitch);

        //Sensors
        pigeon = new Pigeon2(RobotMap.PIGEON_ID, RobotMap.CANIVORE_CAN_NAME);

        elevator = new Elevator(elevatorVerticalMotorMain, RobotMap.mainVerticalFalconR, elevatorVerticalMotorFollower, RobotMap.followerVerticalFalconR, elevatorVerticalEncoder, elevatorHorizontalEncoder, RobotMap.verticalElevatorEncoderR, elevatorHorizontalNeo, elevatorVerticalLimitSwitch, elevatorHorizontalLimitSwitch, elevatorHorizontalHighLimitSwitch);
        claw = new Claw(clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor, clawOpenCloseEncoder, clawTiltNeo, clawTiltEncoder, clawTiltLimitSwitch);

        //Swerve
        kinematics = new SwerveDriveKinematics(
                Constants.Swerve.FL_OFFSET,
                Constants.Swerve.FR_OFFSET,
                Constants.Swerve.BL_OFFSET,
                Constants.Swerve.BR_OFFSET
        );

        swerve = new Swerve(pigeon, kinematics, frontLeft, frontRight, backLeft, backRight);

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                swerve.getRotation2d(),
                swerve.getModulePositions(),
                new Pose2d(),
                Constants.Vision.stateStdDevs,
                Constants.Vision.visionMeasurementStdDevs
        );
        field = new Field2d();

        holonomicDriveController = new DriveController(
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                new PIDController(0, 0, 0)
        );

        //Vision
        photonDriveCamera = new PhotonCamera(RobotMap.PhotonVision_Driver_Cam);
        photonDriverCam = new PhotonDriverCam(photonDriveCamera);

        photonApriltagCamera = new PhotonCamera(RobotMap.PhotonVision_AprilTag_Cam);
        photonApriltags = new PhotonApriltags(photonApriltagCamera, swerve, poseEstimator, field);

        //LEDS
        cANdle = new CANdle(RobotMap.CANdle_ID);
        candleController = new CandleController(cANdle);

        //Teleop Commands
        swerveDriveTeleop = new SwerveDriveTeleop(swerve, elevator, oi.getXboxMain());
        autoAlignment = new AutoAlignment(swerve, poseEstimator, oi.getXboxMain(), field);
        intakeTeleop = new IntakeTeleop(claw, elevator, oi.getXboxMain(), oi.getXboxCo());
        elevatorTeleop = new ElevatorTeleop(elevator, claw, oi.getXboxCo());

        //Buttons
        resetGyroBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_Y);
        alignLeftBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_BUMPER_LEFT);
        alignRightBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_BUMPER_RIGHT);

        candleYellowBtn = new TitanButton(oi.getXboxCo(), OI.XBOX_Y);
        candlePurpleBtn = new TitanButton(oi.getXboxCo(), OI.XBOX_X);

        //Auto Commands
        trajectoryManager = new TrajectoryManager(swerve, field, holonomicDriveController, poseEstimator, claw, elevator);

        //Driver Profile Selector
        profileChooser = new SendableChooser<>();
        profileChooser.setDefaultOption("Driver1", Enums.DriverProfiles.DRIVER1);
        profileChooser.addOption("Driver2", Enums.DriverProfiles.DRIVER2);
        SmartDashboard.putData("Profile Chooser", profileChooser);

        //Autonomous Selector
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("DropAndMobility", trajectoryManager.getCommand("DropAndMobility"));
        autoChooser.addOption("CubeAndChargeBack", trajectoryManager.getCommand("CubeAndChargeBack", 2, 1));
        autoChooser.addOption("DropAndCharge", trajectoryManager.getCommand("DropAndCharge"));
        autoChooser.addOption("2PieceBump", trajectoryManager.getCommand("2PieceBump", 2, 1));
        autoChooser.addOption("2.5PieceNoBalTurns", trajectoryManager.getCommand("2.5PieceNoBalTurns"));
//        autoChooser.addOption("3PieceAuton", trajectoryManager.getCommand("3PieceAuton"));
//        autoChooser.addOption("3PieceAutonV2", trajectoryManager.getCommand("3PieceAutonV2"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //Create Button Bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Main Driver
        resetGyroBtn.onTrue(Commands.runOnce(swerve::zeroRotation));
        alignLeftBtn.whileTrue(Commands.runOnce(() -> autoAlignment.setState(Enums.GridPositions.LEFT)));
        alignRightBtn.whileTrue(Commands.runOnce(() -> autoAlignment.setState(Enums.GridPositions.RIGHT)));

        // Co Driver
        candleYellowBtn.onTrue(Commands.runOnce(() -> candleController.setState(Enums.CANdleState.YELLOW)));
        candlePurpleBtn.onTrue(Commands.runOnce(() -> candleController.setState(Enums.CANdleState.PURPLE)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
