package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autonomous.TestTraj;
import frc.robot.commands.teleop.AutoBalanceTeleop;
import frc.robot.commands.teleop.SwerveAlignment;
import frc.robot.commands.teleop.SwerveDriveTeleop;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.DriveController;
import frc.robot.utils.Enums;
import frc.robot.wrappers.control.OI;
import frc.robot.wrappers.control.TitanButton;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;
import frc.robot.wrappers.sensors.vision.Limelight;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    //OI
    public final OI oi;

    //Motors
    public final TalonFX frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public final TalonFX frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    public final CANcoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    //Elevator
    public final TalonFX elevatorVerticalMotor;
    public final TitanMAX elevatorHorizontalNeo;
    public SparkMaxPIDController horizontalExtensionPID;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    //Claw
    public final TitanSRX clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor;
    public final CANCoder clawOpenCloseEncoder;
//    public final TitanMAX clawTiltNeo;

    //Swerve
    public final SwerveModule frontLeft, frontRight, backLeft, backRight;
    public final SwerveDriveKinematics kinematics;
    public final SwerveDriveOdometry odometry;
    public final DriveController holonomicDriveController;
    public final Field2d field;

    //PDH
    public final PowerDistribution powerDistribution;

    //Sensors
    public final Pigeon2 pigeon;
    public final ColorSensorV3 clawColorSensor;

    //Vision
    public final Limelight limeLight;
    public final PhotonCamera camera;
    public final PhotonVision photonVision;

    //Candle
    public final CANdle cANdle;
    public final CandleController candleController;

    //SubSystems
    public final Swerve swerve;
//    public final Elevator elevator;
//    public final Claw claw;

    //Teleop Commands
    public final SwerveDriveTeleop swerveDriveTeleop;
    public final AutoBalanceTeleop autoBalanceTeleop;
    public final SwerveAlignment swerveAlignment;
//    public final IntakeTeleop intakeTeleop;
//    public final ElevatorTeleop elevatorTeleop;
//    public final DropGamePieceTeleop dropGamePieceTeleop;

    //Buttons
        //Main Driver
    public final TitanButton resetGyroBtn, autoBalanceBtn, elevatorControlBtn, autoAlignBtn;
        //Co Driver
    public final TitanButton dropGamePieceBtn, candleYellowBtn, candlePurpleBtn;

    //Autonomous Commands
//    public final TrajectoryManager trajectoryManager;

    //SmartDashboard
    public final SendableChooser<Profiler.Profiles> profileChooser;

    public RobotContainer() {
        //OI
        oi = new OI();

        //Power Distribution Hub
        powerDistribution = new PowerDistribution(RobotMap.POWER_DISTRIBUTION_HUB, PowerDistribution.ModuleType.kRev);
        powerDistribution.clearStickyFaults();

        //Swerve Drive Motors
        frontLeftDrive = new TalonFX(RobotMap.frontLeftDrive, RobotMap.CANIVORE_CAN_NAME);
        frontLeftDrive.setInverted(RobotMap.frontLeftDriveR);
        frontRightDrive = new TalonFX(RobotMap.frontRightDrive, RobotMap.CANIVORE_CAN_NAME);
        frontRightDrive.setInverted(RobotMap.frontRightDriveR);
        backLeftDrive = new TalonFX(RobotMap.backLeftDrive, RobotMap.CANIVORE_CAN_NAME);
        backLeftDrive.setInverted(RobotMap.backLeftDriveR);
        backRightDrive = new TalonFX(RobotMap.backRightDrive, RobotMap.CANIVORE_CAN_NAME);
        backRightDrive.setInverted(RobotMap.backRightDriveR);

        //Swerve Turning Motors
        frontLeftTurn = new TalonFX(RobotMap.frontLeftTurn, RobotMap.CANIVORE_CAN_NAME);
        frontLeftTurn.setInverted(RobotMap.frontLeftTurnR);
        frontRightTurn = new TalonFX(RobotMap.frontRightTurn, RobotMap.CANIVORE_CAN_NAME);
        frontRightTurn.setInverted(RobotMap.frontRightTurnR);
        backLeftTurn = new TalonFX(RobotMap.backLeftTurn, RobotMap.CANIVORE_CAN_NAME);
        backLeftTurn.setInverted(RobotMap.backLeftTurnR);
        backRightTurn = new TalonFX(RobotMap.backRightTurn, RobotMap.CANIVORE_CAN_NAME);
        backLeftTurn.setInverted(RobotMap.backRightTurnR);

        //Swerve CANCoders
        frontLeftEncoder = new CANcoder(RobotMap.frontLeftEncoder, RobotMap.CANIVORE_CAN_NAME);
        frontRightEncoder = new CANcoder(RobotMap.frontRightEncoder, RobotMap.CANIVORE_CAN_NAME);
        backLeftEncoder = new CANcoder(RobotMap.backLeftEncoder, RobotMap.CANIVORE_CAN_NAME);
        backRightEncoder = new CANcoder(RobotMap.backRightEncoder, RobotMap.CANIVORE_CAN_NAME);

        //Swerve Modules
        //TODO: TUNE THESE / They need to be turned facing the wanted "front" direction then measure the values in smartdashboard
        frontLeft = new SwerveModule(frontLeftDrive, frontLeftTurn, frontLeftEncoder, 0.603);
        frontRight = new SwerveModule(frontRightDrive, frontRightTurn, frontRightEncoder, -0.924);
        backLeft = new SwerveModule(backLeftDrive, backLeftTurn, backLeftEncoder, 0.02);
        backRight = new SwerveModule(backRightDrive, backRightTurn, backRightEncoder, 0.644);

        //Elevator Motors
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.2;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 5;

//        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
//        hardwareLimitSwitchConfigs.ReverseLimitEnable = true;
//        hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
//        hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        elevatorVerticalMotor = new TalonFX(RobotMap.leftVerticalFalcon, RobotMap.CANIVORE_CAN_NAME);
        TalonFXConfiguration VEConfig = new TalonFXConfiguration();
        VEConfig.Slot0.kP = .15; //TODO: TUNE ALL OF THESE
        VEConfig.Slot0.kI = 0;
        VEConfig.Slot0.kD = 0;
        VEConfig.Slot0.kS = 0.07;
        VEConfig.ClosedLoopRamps = closedLoopRampsConfigs;
        VEConfig.MotionMagic = motionMagicConfigs;
        VEConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//        VEConfig.HardwareLimitSwitch = hardwareLimitSwitchConfigs;
        elevatorVerticalMotor.getConfigurator().apply(VEConfig);



        clawMainWheelsMotor = new TitanSRX(RobotMap.clawMainWheelsMotor, RobotMap.clawMainWheelsMotorR);
        clawFollowerWheelsMotor = new TitanSRX(RobotMap.clawFollowerWheelsMotor, RobotMap.clawFollowerWheelsMotorR);
        clawOpenCloseMotor = new TitanSRX(RobotMap.clawOpenCloseMotor, RobotMap.clawOpenCloseMotorR);
        clawOpenCloseEncoder = new CANCoder(RobotMap.clawOpenCloseEncoder);

        elevatorHorizontalNeo = new TitanMAX(RobotMap.horizontalElevatorNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
        horizontalExtensionPID = elevatorHorizontalNeo.getPIDController();
        horizontalExtensionPID.setFeedbackDevice(elevatorHorizontalNeo.getAlternateEncoder(8192));
        kP = 1;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = .1;
        kMinOutput = -.1;
        horizontalExtensionPID.setP(kP);
        horizontalExtensionPID.setI(kI);
        horizontalExtensionPID.setD(kD);
        horizontalExtensionPID.setIZone(kIz);
        horizontalExtensionPID.setFF(kFF);
        horizontalExtensionPID.setOutputRange(kMinOutput, kMaxOutput);
//        clawTiltNeo = new TitanMAX(RobotMap.clawTiltNeo, CANSparkMaxLowLevel.MotorType.kBrushless);

        //Sensors
        pigeon = new Pigeon2(RobotMap.PIGEON_ID, RobotMap.CANIVORE_CAN_NAME);
        clawColorSensor = new ColorSensorV3(RobotMap.CLAW_COLOR_SENSOR);

//        elevator = new Elevator(elevatorVerticalMotor, elevatorHorizontalNeo);
//        claw = new Claw(clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor, clawOpenCloseEncoder, clawTiltNeo, clawColorSensor);

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
        limeLight = new Limelight();
        camera = new PhotonCamera(RobotMap.PhotonVision_AprilTag_Cam);
        photonVision = new PhotonVision(camera);

        //LEDS
        cANdle = new CANdle(RobotMap.CANdle_ID);
        candleController = new CandleController(cANdle);

        //Teleop Commands
        swerveDriveTeleop = new SwerveDriveTeleop(swerve, oi.getXboxMain());
        autoBalanceTeleop = new AutoBalanceTeleop(swerve, pigeon);
        swerveAlignment = new SwerveAlignment(swerve, limeLight, photonVision);
//        intakeTeleop = new IntakeTeleop(claw, oi.getXboxMain());
//        elevatorTeleop = new ElevatorTeleop(elevator, oi.getXboxMain());
//        dropGamePieceTeleop = new DropGamePieceTeleop(claw, elevator, candleController);

        //Buttons
        resetGyroBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_BTN_SELECT);
        elevatorControlBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_A);
        autoBalanceBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_X);
        autoAlignBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_Y);

        dropGamePieceBtn = new TitanButton(oi.getXboxCo(), OI.XBOX_B);
        candleYellowBtn = new TitanButton(oi.getXboxCo(), OI.XBOX_Y);
        candlePurpleBtn = new TitanButton(oi.getXboxCo(), OI.XBOX_X);

        //Auto Commands
//        trajectoryManager = new TrajectoryManager(swerve, holonomicDriveController, odometry, field, claw);

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

        autoAlignBtn.onTrue(swerveAlignment);

        // Co Driver

//        dropGamePieceBtn.onTrue(dropGamePieceTeleop);

        candleYellowBtn.onTrue(new InstantCommand(() -> candleController.setState(Enums.CANdleState.YELLOW)));

        candlePurpleBtn.onTrue(new InstantCommand(() -> candleController.setState(Enums.CANdleState.PURPLE)));

    }

    public Command getAutonomousCommand() {
        TestTraj testTraj = new TestTraj(swerve, kinematics, odometry);
        PathPlannerTrajectory path = PathPlanner.loadPath("auto1", Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION, false);
        return testTraj.followPPTrajectory(path, true);
//        return trajectoryManager.getSelectedPath();
    }
}
