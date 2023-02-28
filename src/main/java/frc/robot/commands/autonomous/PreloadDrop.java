package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Enums;

public class PreloadDrop extends SequentialCommandGroup {

    Swerve swerve;
    Claw claw;
    Elevator elevator;

    public PreloadDrop(Swerve swerve, Claw claw, Elevator elevator) {
        this.swerve = swerve;
        this.claw = claw;
        this.elevator = elevator;

        addRequirements(swerve);

        addCommands(
                new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_HOLDING)),
                new InstantCommand(() -> elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY)),
                new WaitCommand(.5),
                new InstantCommand(() -> elevator.setState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH)),
                new WaitCommand(2),
                new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_DROP)),
                new WaitCommand(1),
                new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_OUTTAKE)),
                new WaitCommand(.5),
                new InstantCommand(() -> claw.setState(Enums.ClawState.CLAW_STANDBY)),
                new InstantCommand(() -> elevator.setState(Enums.ElevatorState.ELEVATOR_STANDBY)),


                new InstantCommand(() -> swerve.setAngle(180)),
//                new InstantCommand(swerve::resetDriveEncoders),
                new DriveStraight(swerve, 225000, -1),
                new DriveStrafe(swerve, 100000, 1),
//                new DriveStraight(swerve, 60000, 1)
                new AutoBalance(swerve)

//                new DriveStraight(swerve, 20000, -1)
        );
    }


}
