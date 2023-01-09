package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.DriveTrainTeleop;
import frc.robot.profiler.Profile;
import frc.robot.profiler.profiles.Driver1;
import frc.robot.profiler.profiles.Driver2;
import frc.robot.subsystems.TankDrive;
import frc.robot.wrappers.control.OI;
import frc.robot.wrappers.motors.TitanFX;

public class RobotContainer {
    //TODO: FOLLOW THIS FORMAT IN THE CONSTRUCTOR WHEN DECLARING THESE VARIABLES

    //Motors
    public final TitanFX leftFrontFX, leftRearFX, rightFrontFX, rightRearFX;

    //Compressor
    public final Compressor compressor;

    //Sensors
    public final Pigeon2 pigeon;

    //Solenoids
    public final Solenoid shifterSolenoid;

    //SubSystems
    public final TankDrive driveTrain;

    //Teleop Commands
    public final DriveTrainTeleop driveTrainTeleopCmd;

    //Autonomous Commands

    //SmartDashboard
    public final SendableChooser<Command> autoChooser;
    public final SendableChooser<Profile> profileChooser;


    public RobotContainer() {
        //Drivetrain Motors
        leftFrontFX = new TitanFX(RobotMap.LEFT_TALON_FRONT, RobotMap.REVERSED_LF_TALON);
        leftRearFX = new TitanFX(RobotMap.LEFT_TALON_BACK, RobotMap.REVERSED_LB_TALON);
        rightFrontFX = new TitanFX(RobotMap.RIGHT_TALON_FRONT, RobotMap.REVERSED_RF_TALON);
        rightRearFX = new TitanFX(RobotMap.RIGHT_TALON_BACK, RobotMap.REVERSED_RB_TALON);
        //DT Followers
        leftRearFX.follow(leftFrontFX);
        rightRearFX.follow(rightFrontFX);

        //Compressor
        compressor = new Compressor(RobotMap.PNEUMATICS_HUB_ID, PneumaticsModuleType.CTREPCM);

        //Sensors
        pigeon = new Pigeon2(RobotMap.PIGEON_ID);

        //Solenoids
        shifterSolenoid = new Solenoid(RobotMap.PNEUMATICS_HUB_ID, PneumaticsModuleType.CTREPCM, RobotMap.GEAR_SHIFT_SOLENOID);

        //Subsystems
        driveTrain = new TankDrive(leftFrontFX, rightFrontFX, pigeon, shifterSolenoid, compressor);

        //Teleop Commands
        driveTrainTeleopCmd = new DriveTrainTeleop(driveTrain, OI::getXboxLeftTriggerMain, OI::getXboxRightTriggerMain, OI::getXboxLeftXMain,false);

        //SmartDashboard
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Auto1", null);
        autoChooser.addOption("Auto2", null);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        profileChooser = new SendableChooser<>();
        profileChooser.setDefaultOption("Driver1", new Driver1());
        profileChooser.addOption("Driver2", new Driver2());
        SmartDashboard.putData("Profile Chooser", profileChooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
