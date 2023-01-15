package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import frc.robot.commands.teleop.SwerveDriveTeleop;
import frc.robot.profiler.Profile;
import frc.robot.profiler.profiles.Driver1;
import frc.robot.profiler.profiles.Driver2;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.wrappers.control.OI;
import frc.robot.wrappers.control.TitanButton;
import frc.robot.wrappers.motors.TitanFX;
import frc.robot.wrappers.motors.TitanSRX;

public class RobotContainer {
    //OI
    public final OI oi;

    //Motors
    public final TitanFX frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public final TitanFX frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    public final CANCoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    //Elevator
    public final TitanFX mainElevatorMotor, followerElevatorMotor;
    public final TitanSRX clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor;
    public final CANSparkMax elevatorHorizontalNeo, clawTiltNeo;
    public final TitanSRX verticalElevatorSRXMAG;

    //Swerve
    public final SwerveModule frontLeft, frontRight, backLeft, backRight;
    public final SwerveDriveKinematics kinematics;
    public final SwerveDriveOdometry odometry;
    public final SwerveDrivePoseEstimator poseEstimator;
    public final HolonomicDriveController holonomicDriveController;
    public final Field2d field;

    //PDH
    public final PowerDistribution powerDistribution;

    //Sensors
    public final Pigeon2 pigeon;

    //SubSystems
    public final Swerve swerve;
    public final Elevator elevator;
    public final Claw claw;

    //Teleop Commands
    public final SwerveDriveTeleop swerveDriveTeleop;
    public final AutoBalanceTeleop autoBalanceTeleop;

    //Buttons
    public final TitanButton resetGyroBtn, autoBalanceBtn, elevatorControlBtn;

    //Autonomous Commands
    public final TrajectoryManager trajectoryManager;

    //SmartDashboard
    public final SendableChooser<Command> autoChooser;
    public final SendableChooser<Profile> profileChooser;


    public RobotContainer() {
        //OI
        oi = new OI();

        //Power Distribution Hub
        powerDistribution = new PowerDistribution(RobotMap.POWER_DISTRIBUTION_HUB, PowerDistribution.ModuleType.kCTRE);
        powerDistribution.clearStickyFaults();

        //Swerve Drive Motors
        frontLeftDrive = new TitanFX(RobotMap.frontLeftDrive, RobotMap.frontLeftDriveR);
        frontRightDrive = new TitanFX(RobotMap.frontRightDrive, RobotMap.frontRightDriveR);
        backLeftDrive = new TitanFX(RobotMap.backLeftDrive, RobotMap.backLeftDriveR);
        backRightDrive = new TitanFX(RobotMap.backRightDrive, RobotMap.backRightDriveR);

        //Swerve Turning Motors
        frontLeftTurn = new TitanFX(RobotMap.frontLeftTurn, RobotMap.frontLeftTurnR);
        frontRightTurn = new TitanFX(RobotMap.frontRightTurn, RobotMap.frontRightTurnR);
        backLeftTurn = new TitanFX(RobotMap.backLeftTurn, RobotMap.backLeftTurnR);
        backRightTurn = new TitanFX(RobotMap.backRightTurn, RobotMap.backRightTurnR);

        //Swerve CANCoders
        frontLeftEncoder = new CANCoder(RobotMap.frontLeftEncoder);
        frontRightEncoder = new CANCoder(RobotMap.frontRightEncoder);
        backLeftEncoder = new CANCoder(RobotMap.backLeftEncoder);
        backRightEncoder = new CANCoder(RobotMap.backRightEncoder);

        //Swerve Modules
        //TODO: TUNE THESE / They need to be turned facing the wanted "front" direction then measure the values in smartdashboard
        frontLeft = new SwerveModule(frontLeftDrive, frontLeftTurn, frontLeftEncoder, 108.46);
        frontRight = new SwerveModule(frontRightDrive, frontRightTurn, frontRightEncoder, -166.38);
        backLeft = new SwerveModule(backLeftDrive, backLeftTurn, backLeftEncoder, 3.6);
        backRight = new SwerveModule(backRightDrive, backRightTurn, backRightEncoder, 115.94);

        //Elevator Motors
        mainElevatorMotor = new TitanFX(RobotMap.leftVerticalFalcon, RobotMap.leftElevatorMotorR);
        followerElevatorMotor = new TitanFX(RobotMap.rightVerticalFalcon, RobotMap.rightElevatorMotorR);
        clawMainWheelsMotor = new TitanSRX(RobotMap.clawMainWheelsMotor, RobotMap.clawMainWheelsMotorR);
        clawFollowerWheelsMotor = new TitanSRX(RobotMap.clawFollowerWheelsMotor, RobotMap.clawFollowerWheelsMotorR);
        clawOpenCloseMotor = new TitanSRX(RobotMap.clawOpenCloseMotor, RobotMap.clawOpenCloseMotorR);
        elevatorHorizontalNeo = new CANSparkMax(RobotMap.horizontalElevatorNeo, CANSparkMaxLowLevel.MotorType.kBrushless);
        clawTiltNeo = new CANSparkMax(RobotMap.clawTiltNeo, CANSparkMaxLowLevel.MotorType.kBrushless);

        //Elevator Encoders
        //Plug Mag encoders into SRX and we will access through can
        verticalElevatorSRXMAG = new TitanSRX(RobotMap.verticalElevatorSRXMAG, false);

        //Sensors
        pigeon = new Pigeon2(RobotMap.PIGEON_ID);

        elevator = new Elevator(mainElevatorMotor, followerElevatorMotor, elevatorHorizontalNeo, verticalElevatorSRXMAG);
        claw = new Claw(clawMainWheelsMotor, clawFollowerWheelsMotor, clawOpenCloseMotor, clawTiltNeo);

        //Swerve
        kinematics = new SwerveDriveKinematics(
                new Translation2d(0.31653734, 0.19324828), //front left //TODO: TUNE THESE
                new Translation2d(-0.31653734, 0.41000172), // back left
                new Translation2d(0.28671266, -0.19324828), // front right
                new Translation2d(-0.28671266, -0.41000172)); //back right //in meters, swerve modules relative to the center of robot

        swerve = new Swerve(pigeon, kinematics, frontLeft, frontRight, backLeft, backRight);
        odometry = new SwerveDriveOdometry(kinematics, swerve.getRotation2d(), swerve.getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, swerve.getRotation2d(), swerve.getModulePositions(), odometry.getPoseMeters()); // this will only be used when we get pose input from LL and then we will update odometry with it.
        field = new Field2d();
        holonomicDriveController = new HolonomicDriveController(
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                new ProfiledPIDController(
                        10, -0.003, 0,
                        new TrapezoidProfile.Constraints(Constants.Swerve.TRAJ_MAX_SPEED, Constants.Swerve.TRAJ_MAX_ACCELERATION)
                ));

        //Teleop Commands
        swerveDriveTeleop = new SwerveDriveTeleop(swerve, oi.getXboxMain());
        autoBalanceTeleop = new AutoBalanceTeleop(swerve, pigeon);

        //Buttons
        resetGyroBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_Y);
        autoBalanceBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_BUMPER_RIGHT);
        elevatorControlBtn = new TitanButton(oi.getXboxMain(), OI.XBOX_A);

        //Auto Commands
        trajectoryManager = new TrajectoryManager(swerve, holonomicDriveController, odometry, field);

        //SmartDashboard
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Auto1", trajectoryManager.getCommand("pathing"));
        autoChooser.addOption("Auto2", null);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        profileChooser = new SendableChooser<>();
        profileChooser.setDefaultOption("Driver1", new Driver1());
        profileChooser.addOption("Driver2", new Driver2());
        SmartDashboard.putData("Profile Chooser", profileChooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        resetGyroBtn.onTrue(new InstantCommand(swerve::zeroRotation));
        autoBalanceBtn.onTrue(autoBalanceTeleop);
//        elevatorControlBtn.onTrue(new InstantCommand(() -> {
//            int currentState = elevator.getCurrentState().ordinal();
//            if (currentState == Enums.ElevatorState.values().length-1) {
//                currentState = 0;
//            } else {
//                currentState++;
//            }
//            elevator.setState(Enums.ElevatorState.values()[currentState]);
//        }));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
