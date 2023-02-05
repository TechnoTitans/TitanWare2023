package frc.robot;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.commands.teleop.AutoBalanceTeleop;
import frc.robot.commands.teleop.SwerveAlignment;
import frc.robot.commands.teleop.SwerveDriveTeleop;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.DriveController;
import frc.robot.utils.Enums;
import frc.robot.wrappers.control.OI;
import frc.robot.wrappers.control.TitanButton;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanSRX;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    //OI
    public final OI oi;

    //Motors
    public final TitanFX frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public final TitanFX frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    public final CANCoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    //Elevator
    public final TitanFX mainElevatorMotor;
    public final TitanSRX clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor;
//    public final CANSparkMax elevatorHorizontalNeo, -clawTiltNeo;

    //Claw

    //Swerve
    public final SwerveModule frontLeft, frontRight, backLeft, backRight;
    public final SwerveDriveKinematics kinematics;
    public final SwerveDriveOdometry odometry;
//    public final HolonomicDriveController holonomicDriveController;
    public final DriveController holonomicDriveController;
    public final Field2d field;

    //PDH
    public final PowerDistribution powerDistribution;

    //Sensors
    public final Pigeon2 pigeon;

    //Vision
    public final Limelight limelight;
    public final PhotonCamera camera;
    public final PhotonVision photonVision;

    //Candle
    public final CANdle cANdle;
    public final CandleController candleController;

    //SubSystems
    public final Swerve swerve;
    //public final Elevator elevator;
    public final Claw claw;

    //Teleop Commands
    public final SwerveDriveTeleop swerveDriveTeleop;
    public final AutoBalanceTeleop autoBalanceTeleop;
    public final SwerveAlignment swerveAlignment;

    //Buttons
        //Main Driver
    public final TitanButton resetGyroBtn, autoBalanceBtn, elevatorControlBtn, autoAlignBtn;
        //Co Driver
    public final TitanButton candleYellowBtn, candlePurpleBtn;


    //Autonomous Commands
    public final TrajectoryManager trajectoryManager;

    //SmartDashboard
    public final SendableChooser<Profiler.Profiles> profileChooser;

    public RobotContainer() {
        //OI
        oi = new OI();

        //Power Distribution Hub
        powerDistribution = new PowerDistribution(RobotMap.POWER_DISTRIBUTION_HUB, PowerDistribution.ModuleType.kCTRE);
        powerDistribution.clearStickyFaults();

        //Swerve Drive Motors
        frontLeftDrive = new TitanFX(RobotMap.frontLeftDrive, RobotMap.frontLeftDriveR, RobotMap.CANIVORE_CAN_NAME);
        frontRightDrive = new TitanFX(RobotMap.frontRightDrive, RobotMap.frontRightDriveR, RobotMap.CANIVORE_CAN_NAME);
        backLeftDrive = new TitanFX(RobotMap.backLeftDrive, RobotMap.backLeftDriveR, RobotMap.CANIVORE_CAN_NAME);
        backRightDrive = new TitanFX(RobotMap.backRightDrive, RobotMap.backRightDriveR, RobotMap.CANIVORE_CAN_NAME);

        //Swerve Turning Motors
        frontLeftTurn = new TitanFX(RobotMap.frontLeftTurn, RobotMap.frontLeftTurnR, RobotMap.CANIVORE_CAN_NAME);
        frontRightTurn = new TitanFX(RobotMap.frontRightTurn, RobotMap.frontRightTurnR, RobotMap.CANIVORE_CAN_NAME);
        backLeftTurn = new TitanFX(RobotMap.backLeftTurn, RobotMap.backLeftTurnR, RobotMap.CANIVORE_CAN_NAME);
        backRightTurn = new TitanFX(RobotMap.backRightTurn, RobotMap.backRightTurnR, RobotMap.CANIVORE_CAN_NAME);

        //Swerve CANCoders
        frontLeftEncoder = new CANCoder(RobotMap.frontLeftEncoder, RobotMap.CANIVORE_CAN_NAME);
        frontRightEncoder = new CANCoder(RobotMap.frontRightEncoder, RobotMap.CANIVORE_CAN_NAME);
        backLeftEncoder = new CANCoder(RobotMap.backLeftEncoder, RobotMap.CANIVORE_CAN_NAME);
        backRightEncoder = new CANCoder(RobotMap.backRightEncoder, RobotMap.CANIVORE_CAN_NAME);

        //Swerve Modules
        //TODO: TUNE THESE / They need to be turned facing the wanted "front" direction then measure the values in smartdashboard
        frontLeft = new SwerveModule(frontLeftDrive, frontLeftTurn, frontLeftEncoder, 108.46);
        frontRight = new SwerveModule(frontRightDrive, frontRightTurn, frontRightEncoder, -166.38);
        backLeft = new SwerveModule(backLeftDrive, backLeftTurn, backLeftEncoder, 3.6);
        backRight = new SwerveModule(backRightDrive, backRightTurn, backRightEncoder, 115.94);

        //Elevator Motors
        mainElevatorMotor = new TitanFX(RobotMap.leftVerticalFalcon, RobotMap.leftElevatorMotorR);
        clawMainWheelsMotor = new TitanSRX(RobotMap.clawMainWheelsMotor, RobotMap.clawMainWheelsMotorR);
        clawFollowerWheelsMotor = new TitanSRX(RobotMap.clawFollowerWheelsMotor, RobotMap.clawFollowerWheelsMotorR);
        clawOpenCloseMotor = new TitanSRX(RobotMap.clawOpenCloseMotor, RobotMap.clawOpenCloseMotorR);
        //elevatorHorizontalNeo = new CANSparkMax(RobotMap.horizontalElevatorNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
//        clawTiltNeo = new CANSparkMax(RobotMap.clawTiltNeo, CANSparkMaxLowLevel.MotorType.kBrushless);

        //Sensors
        pigeon = new Pigeon2(RobotMap.PIGEON_ID, RobotMap.CANIVORE_CAN_NAME);

        //elevator = new Elevator(mainElevatorMotor, elevatorHorizontalNeo);
        claw = new Claw(clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor);

        //Swerve
        kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.Swerve.WHEEL_BASE/2, Constants.Swerve.TRACK_WIDTH/2), //front left //TODO: TUNE THESE
                new Translation2d(-Constants.Swerve.WHEEL_BASE/2, Constants.Swerve.TRACK_WIDTH/2), // back left
                new Translation2d(Constants.Swerve.WHEEL_BASE/2, -Constants.Swerve.TRACK_WIDTH/2), // front right
                new Translation2d(-Constants.Swerve.WHEEL_BASE/2, -Constants.Swerve.TRACK_WIDTH/2)); //back right //in meters, swerve modules relative to the center of robot

        swerve = new Swerve(pigeon, kinematics, frontLeft, frontRight, backLeft, backRight);
        odometry = new SwerveDriveOdometry(kinematics, swerve.getRotation2d(), swerve.getModulePositions());
        field = new Field2d();

        holonomicDriveController = new DriveController(
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new PIDController(0.12, 0, 0)
        );

        //Vision
        limelight = new Limelight();
        camera = new PhotonCamera(RobotMap.PhotonVision_AprilTag_Cam);
        photonVision = new PhotonVision(camera);

        cANdle = new CANdle(RobotMap.CANdle_ID);
        candleController = new CandleController(cANdle);

        //Teleop Commands
        swerveDriveTeleop = new SwerveDriveTeleop(swerve, oi.getXboxMain());
        autoBalanceTeleop = new AutoBalanceTeleop(swerve, pigeon);
        swerveAlignment = new SwerveAlignment(swerve, limelight, photonVision);

        //Buttons
        resetGyroBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_Y);
        autoBalanceBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_X);
        elevatorControlBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_A);
        autoAlignBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_B);

        candleYellowBtn = new TitanButton(oi.getXboxCo(), OI.XBOX_Y);
        candlePurpleBtn = new TitanButton(oi.getXboxCo(), OI.XBOX_X);

        //Auto Commands
        trajectoryManager = new TrajectoryManager(swerve, holonomicDriveController, odometry, field, claw);

        //SmartDashboard
        profileChooser = new SendableChooser<>();
        profileChooser.setDefaultOption("Driver1", Profiler.Profiles.Driver1);
        profileChooser.addOption("Driver2", Profiler.Profiles.Driver2);
        SmartDashboard.putData("Profile Chooser", profileChooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Main Driver
        resetGyroBtn.onTrue(new InstantCommand(swerve::zeroRotation));
//        elevatorControlBtn.onTrue(new InstantCommand(() -> {
//            int currentState = elevator.getCurrentState().ordinal();
//            if (currentState == Enums.ElevatorState.values().length-1) {
//                currentState = 0;
//            } else {
//                currentState++;
//            }
//            elevator.setState(Enums.ElevatorState.values()[currentState]);
//        }));

        elevatorControlBtn.onTrue(new InstantCommand(() -> {
            if (claw.getCurrentState() == Enums.ClawState.CLAW_INTAKING) {
                claw.setState(Enums.ClawState.CLAW_HOLDING);
            } else if (claw.getCurrentState() == Enums.ClawState.CLAW_HOLDING) {
                claw.setState(Enums.ClawState.CLAW_OUTTAKE);
            } else if (claw.getCurrentState() == Enums.ClawState.CLAW_OUTTAKE || claw.getCurrentState() == Enums.ClawState.CLAW_RETRACTED) {
                claw.setState(Enums.ClawState.CLAW_STANDBY);
            } else if (claw.getCurrentState() == Enums.ClawState.CLAW_STANDBY) {
                claw.setState(Enums.ClawState.CLAW_INTAKING);
            }
        }));

        autoAlignBtn.onTrue(swerveAlignment);

        // Co Driver
        candleYellowBtn.onTrue(new InstantCommand(() -> {
            candleController.setState(Enums.CANdleState.YELLOW);
        }));

        candlePurpleBtn.onTrue(new InstantCommand(() -> {
            candleController.setState(Enums.CANdleState.PURPLE);
        }));

    }

    public Command getAutonomousCommand() {
        return trajectoryManager.getSelectedPath();
    }
}
