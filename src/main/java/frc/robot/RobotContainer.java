package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.commands.teleop.ElevatorClawTeleop;
import frc.robot.commands.teleop.SwerveDriveTeleop;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.claw.*;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.utils.auto.*;
import frc.robot.utils.control.DriveToPoseController;
import frc.robot.utils.vision.TitanCamera;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.motors.TitanSparkMAX;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import frc.robot.wrappers.sensors.vision.PhotonVisionIO;
import frc.robot.wrappers.sensors.vision.PhotonVisionIOApriltagsImpl;

import java.util.List;

public class RobotContainer {
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

    //Odometry, PoseEstimator
    public final SwerveDriveOdometry visionIndependentOdometry;
    public final SwerveDrivePoseEstimator poseEstimator;

    //Swerve
    public final SwerveModule frontLeft, frontRight, backLeft, backRight;
    public final SwerveDriveKinematics kinematics;

    //Controllers
    public final DriveController holonomicDriveController;
    public final DriveToPoseController holdPositionDriveController;

    //PDH
    public final PowerDistribution powerDistribution;

    //Sensors
    public final Pigeon2 pigeon2;
    public final Gyro gyro;

    //Vision
    public final TitanCamera photonDriveCamera;
    public final TitanCamera photonFR_Apriltag_R, photonFR_Apriltag_F, photonFL_Apriltag_L, photonBR_Apriltag_B;
    public final PhotonVision photonVision;

    //Candle
    public final CANdle cANdle;
    public final CandleController candleController;

    //SubSystems
    public final Swerve swerve;
    public final ElevatorSimSolver elevatorSimSolver;
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
    public final CustomProfileChooser<Profiler.DriverProfile> profileChooser;
    public final CustomAutoChooser<String, AutoOption> autoChooser;

    public RobotContainer() {
        //Power Distribution Hub
        powerDistribution = new PowerDistribution(RobotMap.POWER_DISTRIBUTION_HUB, PowerDistribution.ModuleType.kRev);
        powerDistribution.clearStickyFaults();

        //Swerve Modules
        frontLeft = switch (Constants.ROBOT) {
            case ROBOT_2023_FALCON_SWERVE -> SwerveModule.Builder.SDSMK4iFalcon500CANCoder(
                    "FrontLeft",
                    new TalonFX(RobotMap.frontLeftDrive, RobotMap.CANIVORE_CAN_NAME),
                    new TalonFX(RobotMap.frontLeftTurn, RobotMap.CANIVORE_CAN_NAME),
                    new CANcoder(RobotMap.frontLeftEncoder, RobotMap.CANIVORE_CAN_NAME),
                    RobotMap.frontLeftDriveR,
                    RobotMap.frontLeftTurnR,
                    0.320556640625,
                    Constants.CURRENT_MODE
            );
            case ROBOT_2023_NEO_SWERVE -> SwerveModule.Builder.SDSMk4iSparkMAX(
                    "FrontLeft",
                    new TitanSparkMAX(RobotMap.frontLeftDrive, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new TitanSparkMAX(RobotMap.frontLeftTurn, CANSparkMaxLowLevel.MotorType.kBrushless),
                    RobotMap.frontLeftDriveR,
                    RobotMap.frontLeftTurnR,
                    0.320556640625,
                    Constants.CURRENT_MODE
            );
        };

        frontRight = switch (Constants.ROBOT) {
            case ROBOT_2023_FALCON_SWERVE -> SwerveModule.Builder.SDSMK4iFalcon500CANCoder(
                    "FrontRight",
                    new TalonFX(RobotMap.frontRightDrive, RobotMap.CANIVORE_CAN_NAME),
                    new TalonFX(RobotMap.frontRightTurn, RobotMap.CANIVORE_CAN_NAME),
                    new CANcoder(RobotMap.frontRightEncoder, RobotMap.CANIVORE_CAN_NAME),
                    RobotMap.frontRightDriveR,
                    RobotMap.frontRightTurnR,
                    0.33251953125,
                    Constants.CURRENT_MODE
            );
            case ROBOT_2023_NEO_SWERVE -> SwerveModule.Builder.SDSMk4iSparkMAX(
                    "FrontRight",
                    new TitanSparkMAX(RobotMap.frontRightDrive, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new TitanSparkMAX(RobotMap.frontRightTurn, CANSparkMaxLowLevel.MotorType.kBrushless),
                    RobotMap.frontRightDriveR,
                    RobotMap.frontRightTurnR,
                    0.33251953125,
                    Constants.CURRENT_MODE
            );
        };

        backLeft = switch (Constants.ROBOT) {
            case ROBOT_2023_FALCON_SWERVE -> SwerveModule.Builder.SDSMK4iFalcon500CANCoder(
                    "BackLeft",
                    new TalonFX(RobotMap.backLeftDrive, RobotMap.CANIVORE_CAN_NAME),
                    new TalonFX(RobotMap.backLeftTurn, RobotMap.CANIVORE_CAN_NAME),
                    new CANcoder(RobotMap.backLeftEncoder, RobotMap.CANIVORE_CAN_NAME),
                    RobotMap.backLeftDriveR,
                    RobotMap.backLeftTurnR,
                    0.0478515625,
                    Constants.CURRENT_MODE
            );
            case ROBOT_2023_NEO_SWERVE -> SwerveModule.Builder.SDSMk4iSparkMAX(
                    "BackLeft",
                    new TitanSparkMAX(RobotMap.backLeftDrive, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new TitanSparkMAX(RobotMap.backLeftTurn, CANSparkMaxLowLevel.MotorType.kBrushless),
                    RobotMap.backLeftDriveR,
                    RobotMap.backLeftTurnR,
                    0.0478515625,
                    Constants.CURRENT_MODE
            );
        };

        backRight = switch (Constants.ROBOT) {
            case ROBOT_2023_FALCON_SWERVE -> SwerveModule.Builder.SDSMK4iFalcon500CANCoder(
                    "BackRight",
                    new TalonFX(RobotMap.backRightDrive, RobotMap.CANIVORE_CAN_NAME),
                    new TalonFX(RobotMap.backRightTurn, RobotMap.CANIVORE_CAN_NAME),
                    new CANcoder(RobotMap.backRightEncoder, RobotMap.CANIVORE_CAN_NAME),
                    RobotMap.backRightDriveR,
                    RobotMap.backRightTurnR,
                    0.283203125,
                    Constants.CURRENT_MODE
            );
            case ROBOT_2023_NEO_SWERVE -> SwerveModule.Builder.SDSMk4iSparkMAX(
                    "BackRight",
                    new TitanSparkMAX(RobotMap.backRightDrive, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new TitanSparkMAX(RobotMap.backRightTurn, CANSparkMaxLowLevel.MotorType.kBrushless),
                    RobotMap.backRightDriveR,
                    RobotMap.backRightTurnR,
                    0.283203125,
                    Constants.CURRENT_MODE
            );
        };

        final SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};

        //Elevator Motors
        elevatorVerticalMotorMain = new TalonFX(RobotMap.mainVerticalFalcon, RobotMap.CANIVORE_CAN_NAME);
        elevatorVerticalMotorFollower = new TalonFX(RobotMap.followerVerticalFalcon, RobotMap.CANIVORE_CAN_NAME);
        elevatorVerticalEncoder = new CANcoder(RobotMap.verticalElevatorEncoder, RobotMap.CANIVORE_CAN_NAME);
        elevatorVerticalLimitSwitch = new DigitalInput(RobotMap.verticalLimitSwitch);

        elevatorHorizontalNeo = new TitanSparkMAX(
                RobotMap.horizontalElevatorNeo, CANSparkMaxLowLevel.MotorType.kBrushless
        );
        elevatorHorizontalEncoder = new CANcoder(RobotMap.horizontalElevatorEncoder);
        elevatorHorizontalLimitSwitch = new DigitalInput(RobotMap.horizontalLimitSwitch);
        elevatorHorizontalHighLimitSwitch = new DigitalInput(RobotMap.horizontalLimitHighSwitch);

        clawTiltNeo = new TitanSparkMAX(RobotMap.clawTiltNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
        clawTiltEncoder = new CANcoder(RobotMap.clawTiltEncoder);
        clawTiltLimitSwitch = new DigitalInput(RobotMap.clawLimitSwitch);

        clawMainWheelsMotor = new TalonSRX(RobotMap.clawMainWheelsMotor);
        clawFollowerWheelsMotor = new TalonSRX(RobotMap.clawFollowerWheelsMotor);
        clawOpenCloseMotor = new TalonSRX(RobotMap.clawOpenCloseMotor);
        clawOpenCloseEncoder = new CANCoder(RobotMap.clawOpenCloseEncoder);

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
            case REAL -> new Gyro(new GyroIOPigeon2(pigeon2), pigeon2);
            case SIM -> new Gyro(new GyroIOSim(pigeon2, kinematics, swerveModules), pigeon2);
            case REPLAY -> new Gyro(new GyroIO() {}, pigeon2);
        };

        elevatorSimSolver = new ElevatorSimSolver(
                elevatorVerticalMotorMain,
                elevatorVerticalMotorFollower,
                elevatorVerticalEncoder,
                elevatorHorizontalEncoder,
                elevatorHorizontalNeo
        );

        //Elevator
        elevator = switch (Constants.CURRENT_MODE) {
            case REAL -> new Elevator(
                    new ElevatorIOReal(
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
                    )
            );
            case SIM -> new Elevator(
                    new ElevatorIOSim(
                            elevatorVerticalMotorMain,
                            RobotMap.mainVerticalFalconR,
                            elevatorVerticalMotorFollower,
                            RobotMap.followerVerticalFalconR,
                            elevatorVerticalEncoder,
                            RobotMap.verticalElevatorEncoderR,
                            elevatorHorizontalEncoder,
                            RobotMap.horizontalElevatorEncoderR,
                            elevatorHorizontalNeo,
                            elevatorVerticalLimitSwitch,
                            elevatorHorizontalLimitSwitch,
                            elevatorSimSolver
                    ),
                    elevatorSimSolver
            );
            case REPLAY -> new Elevator(new ElevatorIO() {}, elevatorSimSolver);
        };

        claw = switch (Constants.CURRENT_MODE) {
            case REAL -> new Claw(new ClawIOStateSpace(
                    clawMainWheelsMotor,
                    clawFollowerWheelsMotor,
                    RobotMap.clawMainWheelsMotorInverted,
                    clawOpenCloseMotor,
                    RobotMap.clawOpenCloseMotorInverted,
                    clawOpenCloseEncoder,
                    clawTiltNeo,
                    clawTiltEncoder
            ));
            case SIM -> new Claw(new ClawIOStateSpaceSim(
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
            case REPLAY -> new Claw(new ClawIO() {});
        };

        //Swerve
        swerve = new Swerve(gyro, kinematics, frontLeft, frontRight, backLeft, backRight);

        final Pose2d initialOdometryPose = new Pose2d();
        visionIndependentOdometry = new SwerveDriveOdometry(
                kinematics, swerve.getYaw(), swerve.getModulePositions(), initialOdometryPose
        );
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                swerve.getYaw(),
                swerve.getModulePositions(),
                initialOdometryPose,
                Constants.Vision.STATE_STD_DEVS,
                Constants.Vision.VISION_MEASUREMENT_STD_DEVS
        );

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
        holdPositionDriveController = new DriveToPoseController(
                new ProfiledPIDController(
                        5, 0, 0,
                        new TrapezoidProfile.Constraints(
                                Constants.Swerve.TRAJECTORY_MAX_SPEED,
                                Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
                        )
                ),
                new ProfiledPIDController(
                        5, 0, 0,
                        new TrapezoidProfile.Constraints(
                                Constants.Swerve.TRAJECTORY_MAX_SPEED,
                                Constants.Swerve.TRAJECTORY_MAX_ACCELERATION
                        )
                ),
                new ProfiledPIDController(
                        1, 0, 0,
                        new TrapezoidProfile.Constraints(
                                Constants.Swerve.TRAJECTORY_MAX_ANGULAR_SPEED,
                                Constants.Swerve.TRAJECTORY_MAX_ANGULAR_ACCELERATION
                        )
                )
        );

        //Vision
        photonDriveCamera = TitanCamera.DRIVER_CAM;
        photonFR_Apriltag_F = TitanCamera.PHOTON_FR_APRILTAG_F;
        photonFR_Apriltag_R = TitanCamera.PHOTON_FR_Apriltag_R;
        photonFL_Apriltag_L = TitanCamera.PHOTON_FL_Apriltag_L;
        photonBR_Apriltag_B = TitanCamera.PHOTON_BR_Apriltag_B;

        final List<TitanCamera> apriltagCameras = List.of(
                photonFR_Apriltag_F,
                photonFR_Apriltag_R,
                photonFL_Apriltag_L,
                photonBR_Apriltag_B
        );

        photonVision = switch (Constants.CURRENT_MODE) {
            case REAL, SIM -> new PhotonVision(
                    new PhotonVisionIOApriltagsImpl(swerve, visionIndependentOdometry, apriltagCameras),
                    swerve, visionIndependentOdometry, poseEstimator, apriltagCameras
            );
            case REPLAY -> new PhotonVision(
                    new PhotonVisionIO() {}, swerve, visionIndependentOdometry, poseEstimator, apriltagCameras
            );
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
        trajectoryManager = new TrajectoryManager(
                swerve,
                holonomicDriveController,
                holdPositionDriveController,
                candleController,
                photonVision,
                claw,
                elevator
        );

        //Driver Profile Selector
        profileChooser = new CustomProfileChooser<>(
                Constants.NetworkTables.PROFILE_TABLE,
                Constants.NetworkTables.PROFILE_PUBLISHER,
                Constants.NetworkTables.PROFILE_SELECTED_SUBSCRIBER,
                Profiler.DriverProfile.DEFAULT
        );
        profileChooser.addOptionsIfNotPresent(Enum::name, List.of(Profiler.DriverProfile.values()));

        //Autonomous Selector
        autoChooser = new CustomAutoChooser<>(
                Constants.NetworkTables.AUTO_TABLE,
                Constants.NetworkTables.AUTO_PUBLISHER,
                Constants.NetworkTables.AUTO_SELECTED_SUBSCRIBER,
                new AutoOption("DropAndMobility")
        );
        //Add paths that are specifically for one competition type here
        autoChooser.addAutoOption(
                new AutoOption(
                        "CubeAndChargeBack",
                        2,
                        1,
                        Constants.CompetitionType.COMPETITION
                )
        );
        autoChooser.addAutoOption(new AutoOption("DropAndCharge", 2, 1, Constants.CompetitionType.COMPETITION));
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
    }

    public Command getAutonomousCommand() {
        final AutoOption selectedAutoOption = autoChooser.getSelected();
        return selectedAutoOption != null
                ? trajectoryManager.getCommand(selectedAutoOption)
                : Commands.waitUntil(() -> !RobotState.isAutonomous());
    }
}
