package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.commands.teleop.ElevatorClawTeleop;
import frc.robot.commands.teleop.SwerveDriveTeleop;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.auto.AutoOption;
import frc.robot.utils.auto.CustomAutoChooser;
import frc.robot.utils.auto.CustomProfileChooser;
import frc.robot.wrappers.leds.CandleController;
import frc.robot.wrappers.sensors.vision.PhotonVision;

import java.util.List;

public class RobotContainer {
    public final PowerDistribution powerDistribution;

    //Vision
    public final PhotonVision photonVision;

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

    public final CustomProfileChooser<Profiler.DriverProfile> profileChooser;
    public final CustomAutoChooser<String, AutoOption> autoChooser;

    public RobotContainer() {
        powerDistribution = new PowerDistribution(RobotMap.POWER_DISTRIBUTION_HUB, PowerDistribution.ModuleType.kRev);
        powerDistribution.clearStickyFaults();

        elevator = new Elevator(HardwareConstants.ELEVATOR);
        claw = new Claw(HardwareConstants.CLAW, elevator::getElevatorPoseState);
        swerve = new Swerve(
                HardwareConstants.FRONT_LEFT_MODULE,
                HardwareConstants.FRONT_RIGHT_MODULE,
                HardwareConstants.BACK_LEFT_MODULE,
                HardwareConstants.BACK_RIGHT_MODULE
        );

//        holonomicDriveController = new DriveController(
//                new PIDController(14, 0, 0),
//                new PIDController(22, 0, 0),
//                new PIDController(12, 0, 0)
//        );

//        holonomicDriveController = new DriveController(
//                new PIDController(6, 0, 0),
//                new PIDController(11, 0, 0),
//                new PIDController(3.2, 0, 0),
//                true,
//                true,
//                true,
//                false
//        );

        //Vision
        photonVision = new PhotonVision(swerve, swerve.getPoseEstimator());

        //LEDs
        CandleController.getInstance();

        //Controllers
        driverController = new CommandXboxController(RobotMap.MainController);
        coDriverController = new CommandXboxController(RobotMap.CoController);

        //Teleop Commands
        swerveDriveTeleop = new SwerveDriveTeleop(swerve, elevator, driverController.getHID());
        elevatorClawTeleop = new ElevatorClawTeleop(elevator, claw);

        //Auto Commands
        trajectoryManager = new TrajectoryManager(
                swerve,
                photonVision
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
                Constants.NetworkTables.AUTO_SELECTED_SUBSCRIBER
        );

//        Add paths that are specifically for one competition type here
//        autoChooser.addAutoOption(
//                new AutoOption(
//                        "CubeAndChargeBack",
//                        2,
//                        1,
//                        Constants.CompetitionType.COMPETITION
//                )
//        );
//        autoChooser.addAutoOption(
//                new AutoOption(
//                        "DropAndCharge", 2, 1, Constants.CompetitionType.COMPETITION
//                )
//        );
//        autoChooser.addAutoOption(
//                new AutoOption(
//                        "2PieceBump", 2, 1, Constants.CompetitionType.COMPETITION
//                )
//        );
//        autoChooser.addAutoOption(
//                new AutoOption(
//                        "2PieceAuto", 2, 1, Constants.CompetitionType.COMPETITION
//                )
//        );
//        autoChooser.addAutoOption(
//                new AutoOption(
//                        "2.5PieceNoBalTurns",
//                        Units.feetToMeters(13),
//                        2 * Units.feetToMeters(13),
//                        Constants.CompetitionType.COMPETITION
//                )
//        );
//        autoChooser.addAutoOption(new AutoOption("3PieceAuton", Constants.CompetitionType.COMPETITION));
//        autoChooser.addAutoOption(new AutoOption("3PieceAutonV2", Constants.CompetitionType.COMPETITION));
//
//        autoChooser.addAutoOption(new AutoOption("2Cube1Cone", Constants.CompetitionType.COMPETITION));

//        autoChooser.addAutoOption(new AutoOption(
//                "NewMethod3Piece",
//                List.of(
//                        new AutoOption.PathOption("NewMethod1"),
//                        new AutoOption.PathOption("NewMethod2"),
//                        new AutoOption.PathOption("NewMethod3")
//                ),
//                Constants.CompetitionType.COMPETITION
//        ));
//
//        autoChooser.addAutoOption(new AutoOption(
//                "OtherMethod3Piece",
//                List.of(
//                        new AutoOption.PathOption("OtherMethod1"),
//                        new AutoOption.PathOption("OtherMethod2")
//                ),
//                Constants.CompetitionType.COMPETITION
//        ));

        //Add the remaining paths automatically
//        autoChooser.addOptionsIfNotPresent(
//                AutoOption::getDescriptiveName,
//                AutoOption::new,
//                PathPlannerUtil.getAllPathPlannerPathNames().stream().sorted().toList()
//        );

        autoChooser.addAutoOption(new AutoOption("CurvyForwardAuto"));
        autoChooser.addAutoOption(new AutoOption("TestAuto"));
    }

    public Command getAutonomousCommand() {
        final AutoOption selectedAutoOption = autoChooser.getSelected();
        return selectedAutoOption != null
                ? trajectoryManager.followAutoCommand(selectedAutoOption)
                : Commands.waitUntil(() -> !RobotState.isAutonomous());
    }
}
