package frc.robot.utils.teleop;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.autoalign.AutoAlignmentV3;
import frc.robot.commands.teleop.ElevatorClawTeleop;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;
import frc.robot.utils.alignment.AlignmentZone;

public class ButtonBindings {
    public static void bindAll(final RobotContainer robotContainer) {
        final CommandXboxController driverController = robotContainer.driverController;
        final CommandXboxController coDriverController = robotContainer.coDriverController;

        final Swerve swerve = robotContainer.swerve;
        final Elevator elevator = robotContainer.elevator;
        final Claw claw = robotContainer.claw;

        driverController.y().onTrue(Commands.runOnce(() -> swerve.zeroRotation(robotContainer.photonVision)));
        //TODO: check that this method of making auto alignment commands still works in ALL cases
        // from limited testing in sim, it does seem to work
//        driverController.leftBumper().whileTrue(
//                new AutoAlignment(swerve, robotContainer.photonVision, driverController.getHID())
//                        .withDesiredAlignmentPosition(AlignmentZone.GenericDesiredAlignmentPosition.LEFT)
//        );
//
//        driverController.rightBumper().whileTrue(
//                new AutoAlignment(swerve, robotContainer.photonVision, driverController.getHID())
//                        .withDesiredAlignmentPosition(AlignmentZone.GenericDesiredAlignmentPosition.RIGHT)
//        );

        driverController.leftBumper().whileTrue(
                new AutoAlignmentV3(
                        robotContainer.swerve,
                        robotContainer.elevator,
                        robotContainer.claw,
                        robotContainer.photonVision,
                        robotContainer.trajectoryManager
                ).withDesiredCommunitySide(AlignmentZone.CommunitySide.RIGHT)
        );

        driverController.rightBumper().whileTrue(
                new AutoAlignmentV3(
                        robotContainer.swerve,
                        robotContainer.elevator,
                        robotContainer.claw,
                        robotContainer.photonVision,
                        robotContainer.trajectoryManager
                ).withDesiredCommunitySide(AlignmentZone.CommunitySide.LEFT)
        );

        // Co Driver
        coDriverController.y().onTrue(
                Commands.runOnce(() -> robotContainer.candleController.setState(Enums.CANdleState.YELLOW))
        );
        coDriverController.x().onTrue(
                Commands.runOnce(() -> robotContainer.candleController.setState(Enums.CANdleState.PURPLE))
        );

        // ElevatorClawTeleop
        // TODO: make these cleaner, more builder methods have been added since original bindings were made
        //  and some, if not most, bindings can be simplified now

        // Driver Controller
        ElevatorClawTeleop.addMapping(
                driverController.a(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withConditionalElevatorState(
                                Enums.ElevatorState.ELEVATOR_CUBE, Enums.ElevatorState.ELEVATOR_STANDBY
                        )
                        .withClawState(Enums.ClawState.CLAW_INTAKING_CONE)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                driverController.b(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(Enums.ElevatorState.ELEVATOR_CUBE, Enums.ClawState.CLAW_ANGLE_CUBE)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                driverController.x(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(Enums.ElevatorClawStateType.INTAKING)
                        .withClawState(Enums.ClawState.CLAW_HOLDING)
                        .waitIfState(Enums.ElevatorState.ELEVATOR_DOUBLE_SUB, 0.3)
                        .withElevatorState(Enums.ElevatorState.ELEVATOR_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                driverController.start(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(Enums.ElevatorState.ELEVATOR_TIPPED_CONE, Enums.ClawState.TIPPED_CONE)
                        .build()
        );

        // CoDriver Controller
        ElevatorClawTeleop.addMapping(
                coDriverController.a(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(Enums.ElevatorClawStateType.SCORING)
                        .withClawState(Enums.ClawState.CLAW_OUTTAKE)
                        .wait(0.7)
                        .withElevatorClawStates(Enums.ElevatorState.ELEVATOR_STANDBY, Enums.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.b(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(Enums.ElevatorState.ELEVATOR_SINGLE_SUB, Enums.ClawState.SINGLE_SUB)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.leftBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(Enums.ClawState.CLAW_ANGLE_SHOOT)
                        .withClawState(Enums.ClawState.CLAW_SHOOT_HIGH)
                        .wait(0.4)
                        .withClawState(Enums.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.rightBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(Enums.ClawState.CLAW_ANGLE_SHOOT)
                        .withClawState(Enums.ClawState.CLAW_SHOOT_LOW)
                        .wait(0.4)
                        .withClawState(Enums.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.leftBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withConditionalNotClawState(Enums.ClawState.CLAW_ANGLE_SHOOT, Enums.ClawState.CLAW_ANGLE_SHOOT)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.rightBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfInState(Enums.ClawState.CLAW_ANGLE_SHOOT)
                        .withClawState(Enums.ClawState.CLAW_DROP)
                        .wait(0.25)
                        .withClawState(Enums.ClawState.CLAW_OUTTAKE_HYBRID)
                        .wait(0.65)
                        .withClawState(Enums.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povUp(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorState(Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH)
                        .wait(0.3)
                        .withClawState(Enums.ClawState.CLAW_DROP)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povRight(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(Enums.ElevatorState.ELEVATOR_EXTENDED_MID, Enums.ClawState.CLAW_DROP)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povDown(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(Enums.ElevatorState.ELEVATOR_STANDBY, Enums.ClawState.CLAW_HOLDING)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povLeft(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorState(Enums.ElevatorState.ELEVATOR_DOUBLE_SUB)
                        .wait(0.1)
                        .withClawState(Enums.ClawState.CLAW_INTAKING_CONE)
                        .build()
        );
    }

    public static void clear() {
        ElevatorClawTeleop.removeAllMappings();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }
}
