package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.commands.teleop.ElevatorClawTeleop;
import frc.robot.commands.teleop.SwerveDriveTeleop;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIOReal;
import frc.robot.subsystems.claw.ClawIOSim;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIOImpl;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSimSolver;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.utils.Enums;
import frc.robot.utils.auto.AutoChooser;
import frc.robot.utils.auto.AutoOption;
import frc.robot.utils.auto.DriveController;
import frc.robot.utils.auto.PathPlannerUtil;
import frc.robot.utils.vision.TitanCamera;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.motors.TitanSparkMAX;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import frc.robot.wrappers.sensors.vision.PhotonVisionIOApriltagsReal;
import frc.robot.wrappers.sensors.vision.PhotonVisionIOApriltagsSim;

import java.util.List;

public class RobotContainer {
    //Motors
    public final TalonFX frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public final TalonFX frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    public final CANcoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    //Elevator
    public final TalonFX elevatorVerticalMotorMain, elevatorVerticalMotorFollower;
    public final CANcoder elevatorVerticalEncoder, elevatorHorizontalEncoder;
    public final TitanSparkMAX elevatorHorizontalNeo;
    public final DigitalInput elevatorVerticalLimitSwitch, elevatorHorizontalLimitSwitch, elevatorHorizontalHighLimitSwitch;

    //Claw
    public final TalonSRX clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor;
    public final CANCoder clawOpenCloseEncoder;
    public final CANcoder clawTiltEncoder;
    public final TitanSparkMAX clawTiltNeo;
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
    public final Pigeon2 pigeon2;
    public final Gyro gyro;

    //Vision
    public final TitanCamera photonDriveCamera, photonFR_Apriltag_R, photonFR_Apriltag_F;
    public final PhotonVision photonVision;

    //Candle
    public final CANdle cANdle;
    public final CandleController candleController;

    //SubSystems
    public final Swerve swerve;
    public final Elevator elevator;
    public final Claw claw;

    //Teleop Commands
    public final SwerveDriveTeleop swerveDriveTeleop;
    public final ElevatorClawTeleop elevatorClawTeleop;

    //Controllers
    public final CommandXboxController driverController, coDriverController;

    //Autonomous Commands
    public final TrajectoryManager trajectoryManager;

    //SmartDashboard
    public final SendableChooser<Enums.DriverProfile> profileChooser;
    public final AutoChooser<String, AutoOption> autoChooser;

    public RobotContainer() {
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
        frontLeft = new SwerveModule(
                new SwerveModuleIOImpl(
                        frontLeftDrive, frontLeftTurn, frontLeftEncoder,
                        RobotMap.frontLeftDriveR, RobotMap.frontLeftTurnR, 0.320556640625
                ),
                "FrontLeft"
        );

        frontRight = new SwerveModule(
                new SwerveModuleIOImpl(
                        frontRightDrive, frontRightTurn, frontRightEncoder,
                        RobotMap.frontRightDriveR, RobotMap.frontRightTurnR, 0.33251953125
                ),
                "FrontRight"
        );

        backLeft = new SwerveModule(
                new SwerveModuleIOImpl(
                        backLeftDrive, backLeftTurn, backLeftEncoder,
                        RobotMap.backLeftDriveR, RobotMap.backLeftTurnR, 0.0478515625
                ),
                "BackLeft"
        );

        backRight = new SwerveModule(
                new SwerveModuleIOImpl(
                        backRightDrive, backRightTurn, backRightEncoder,
                        RobotMap.backRightDriveR, RobotMap.backRightTurnR, 0.283203125
                ),
                "BackRight"
        );

        final SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};

        //Elevator Motors
        elevatorVerticalMotorMain = new TalonFX(RobotMap.mainVerticalFalcon, RobotMap.CANIVORE_CAN_NAME);
        elevatorVerticalMotorFollower = new TalonFX(RobotMap.followerVerticalFalcon, RobotMap.CANIVORE_CAN_NAME);
        elevatorVerticalEncoder = new CANcoder(RobotMap.verticalElevatorEncoder, RobotMap.CANIVORE_CAN_NAME);
        elevatorHorizontalEncoder = new CANcoder(RobotMap.horizontalElevatorEncoder);

        elevatorVerticalLimitSwitch = new DigitalInput(RobotMap.verticalLimitSwitch);
        elevatorHorizontalLimitSwitch = new DigitalInput(RobotMap.horizontalLimitSwitch);
        elevatorHorizontalHighLimitSwitch = new DigitalInput(RobotMap.horizontalLimitHighSwitch);

        clawMainWheelsMotor = new TalonSRX(RobotMap.clawMainWheelsMotor);
        clawFollowerWheelsMotor = new TalonSRX(RobotMap.clawFollowerWheelsMotor);
        clawOpenCloseMotor = new TalonSRX(RobotMap.clawOpenCloseMotor);
        clawOpenCloseEncoder = new CANCoder(RobotMap.clawOpenCloseEncoder);

        elevatorHorizontalNeo = new TitanSparkMAX(RobotMap.horizontalElevatorNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
        clawTiltNeo = new TitanSparkMAX(RobotMap.clawTiltNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
        clawTiltEncoder = new CANcoder(RobotMap.clawTiltEncoder);
        clawTiltLimitSwitch = new DigitalInput(RobotMap.clawLimitSwitch);

        //Swerve Kinematics
        kinematics = new SwerveDriveKinematics(
                Constants.Swerve.FL_OFFSET,
                Constants.Swerve.FR_OFFSET,
                Constants.Swerve.BL_OFFSET,
                Constants.Swerve.BR_OFFSET
        );

        //Sensors
        pigeon2 = new Pigeon2(RobotMap.PIGEON_ID, RobotMap.CANIVORE_CAN_NAME);
        gyro = switch (Constants.CURRENT_MODE) {
            case REAL:
                yield new Gyro(new GyroIOPigeon2(pigeon2), pigeon2);
            case SIM:
                yield new Gyro(new GyroIOSim(pigeon2, kinematics, swerveModules), pigeon2);
            case REPLAY:
                throw new RuntimeException("this isn't possible");
        };

        //Elevator
        elevator = switch (Constants.CURRENT_MODE) {
            case REAL:
                yield new Elevator(new ElevatorIOReal(
                        elevatorVerticalMotorMain,
                        RobotMap.mainVerticalFalconR,
                        elevatorVerticalMotorFollower,
                        RobotMap.followerVerticalFalconR,
                        elevatorVerticalEncoder,
                        elevatorHorizontalEncoder,
                        RobotMap.verticalElevatorEncoderR,
                        elevatorHorizontalNeo,
                        elevatorVerticalLimitSwitch,
                        elevatorHorizontalLimitSwitch
                ));
            case SIM:
                final ElevatorSimSolver elevatorSimSolver = new ElevatorSimSolver(
                        elevatorVerticalMotorMain,
                        elevatorVerticalMotorFollower,
                        elevatorVerticalEncoder,
                        elevatorHorizontalEncoder,
                        elevatorHorizontalNeo
                );

                yield new Elevator(
                        new ElevatorIOSim(
                                elevatorVerticalMotorMain,
                                RobotMap.mainVerticalFalconR,
                                elevatorVerticalMotorFollower,
                                RobotMap.followerVerticalFalconR,
                                elevatorVerticalEncoder,
                                RobotMap.verticalElevatorEncoderR,
                                elevatorHorizontalEncoder,
                                SensorDirectionValue.Clockwise_Positive,
                                elevatorHorizontalNeo,
                                elevatorVerticalLimitSwitch,
                                elevatorHorizontalLimitSwitch,
                                elevatorSimSolver
                        ),
                        elevatorSimSolver
                );
            case REPLAY:
                throw new RuntimeException("this isn't possible");
        };

        claw = switch (Constants.CURRENT_MODE) {
            case REAL:
                yield new Claw(new ClawIOReal(
                        clawMainWheelsMotor,
                        clawFollowerWheelsMotor,
                        RobotMap.clawMainWheelsMotorInverted,
                        clawOpenCloseMotor,
                        RobotMap.clawOpenCloseMotorInverted,
                        clawOpenCloseEncoder,
                        clawTiltNeo,
                        clawTiltEncoder
                ));
            case SIM:
                yield new Claw(new ClawIOSim(
                        clawMainWheelsMotor,
                        clawFollowerWheelsMotor,
                        RobotMap.clawMainWheelsMotorInverted,
                        clawOpenCloseMotor,
                        RobotMap.clawOpenCloseMotorInverted,
                        clawOpenCloseEncoder,
                        clawTiltNeo,
                        clawTiltEncoder,
                        elevator::getElevatorSimState
                ));
            case REPLAY:
                throw new RuntimeException("this isn't possible");
        };

        //Swerve
        swerve = new Swerve(gyro, kinematics, frontLeft, frontRight, backLeft, backRight);

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                swerve.getYaw(),
                swerve.getModulePositions(),
                new Pose2d(),
                Constants.Vision.STATE_STD_DEVS,
                Constants.Vision.VISION_MEASUREMENT_STD_DEVS
        );

        field = new Field2d();

//        holonomicDriveController = new DriveController(
//                new PIDController(14, 0, 0),
//                new PIDController(22, 0, 0),
//                new PIDController(12, 0, 0)
//        );

        //3.4
        holonomicDriveController = new DriveController(
                new PIDController(6, 0, 0),
                new PIDController(11, 0, 0),
                new PIDController(3.2, 0, 0),
                true,
                true,
                true,
                false
        );

        //Vision
        photonDriveCamera = TitanCamera.DRIVER_CAM;
        photonFR_Apriltag_F = TitanCamera.PHOTON_FR_APRILTAG_F;
        photonFR_Apriltag_R = TitanCamera.PHOTON_FR_Apriltag_R;

        final List<TitanCamera> apriltagCameras = List.of(photonFR_Apriltag_R, photonFR_Apriltag_F);
        photonVision = switch (Constants.CURRENT_MODE) {
            case REAL:
                yield new PhotonVision(
                        new PhotonVisionIOApriltagsReal(swerve, poseEstimator, apriltagCameras),
                        swerve, poseEstimator, field
                );
            case SIM:
//                yield new PhotonVision(
//                        new PhotonVisionIOApriltagsSim(swerve, poseEstimator, apriltagCameras),
//                        swerve, poseEstimator, field
//                );
                // yield new real for now as sim is not working, will need to address this sooner rather than later
                //TODO: fix simulated vision (solvePnP/sqPNP doesn't seem to work in sim just yet...
                // or maybe we're doing something wrong as it seemed to work in certain examples before)
                yield new PhotonVision(
                        new PhotonVisionIOApriltagsReal(swerve, poseEstimator, apriltagCameras),
                        swerve, poseEstimator, field
                );
            case REPLAY:
                throw new RuntimeException("this isn't possible");
        };

        //LEDs
        cANdle = new CANdle(RobotMap.CANdle_ID);
        candleController = new CandleController(cANdle);

        //Controllers
        driverController = new CommandXboxController(RobotMap.MainController);
        coDriverController = new CommandXboxController(RobotMap.CoController);

        //Teleop Commands
        swerveDriveTeleop = new SwerveDriveTeleop(swerve, elevator, driverController.getHID());
        elevatorClawTeleop = new ElevatorClawTeleop(elevator, claw);

        //Auto Commands
        trajectoryManager = new TrajectoryManager(swerve, holonomicDriveController, poseEstimator, claw, elevator);

        //Driver Profile Selector
        profileChooser = new SendableChooser<>();
        profileChooser.setDefaultOption("Driver1", Enums.DriverProfile.DRIVER1);
        profileChooser.addOption("Driver2", Enums.DriverProfile.DRIVER2);
        SmartDashboard.putData("Profile Chooser", profileChooser);

        //Autonomous Selector
        autoChooser = new AutoChooser<>();
        //Add paths that are specifically for one competition type here
        autoChooser.setDefaultAutoOption(
                new AutoOption("DropAndMobility", Constants.CompetitionType.COMPETITION)
        );
        autoChooser.addAutoOption(
                new AutoOption(
                        "CubeAndChargeBack",
                        2,
                        1,
                        Constants.CompetitionType.COMPETITION
                )
        );
        autoChooser.addAutoOption(new AutoOption("DropAndCharge", Constants.CompetitionType.COMPETITION));
        autoChooser.addAutoOption(
                new AutoOption(
                        "2PieceBump", 2, 1, Constants.CompetitionType.COMPETITION
                )
        );
        autoChooser.addAutoOption(
                new AutoOption(
                        "2PieceAuto", 2, 1, Constants.CompetitionType.COMPETITION
                )
        );
        autoChooser.addAutoOption(
                new AutoOption(
                        "2.5PieceNoBalTurns",
                        Units.feetToMeters(13),
                        2 * Units.feetToMeters(13),
                        Constants.CompetitionType.COMPETITION
                )
        );
        autoChooser.addAutoOption(new AutoOption("3PieceAuton", Constants.CompetitionType.COMPETITION));
        autoChooser.addAutoOption(new AutoOption("3PieceAutonV2", Constants.CompetitionType.COMPETITION));

        //Add the remaining paths automatically
        autoChooser.addOptionsIfNotPresent(
                AutoOption::pathName,
                AutoOption::new,
                PathPlannerUtil.getAllPathPlannerPathNames().stream().sorted().toList()
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        final AutoOption selectedAutoOption = autoChooser.getSelected();
        return selectedAutoOption != null
                ? trajectoryManager.getCommand(selectedAutoOption)
                : Commands.waitUntil(() -> !RobotState.isAutonomous());
    }
}
