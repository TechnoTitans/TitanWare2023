package frc.robot.utils.teleop;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.autoalign.AutoAlignment;
import frc.robot.commands.autoalign.TrajectoryAutoAlignment;
import frc.robot.commands.teleop.ElevatorClawTeleop;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.alignment.AlignmentZone;

public class ButtonBindings {
    public static void bindAll(final RobotContainer robotContainer) {
        final CommandXboxController driverController = robotContainer.driverController;
        final CommandXboxController coDriverController = robotContainer.coDriverController;

        final Swerve swerve = robotContainer.swerve;
        final Elevator elevator = robotContainer.elevator;
        final Claw claw = robotContainer.claw;

        driverController.y().onTrue(Commands.runOnce(() -> swerve.zeroRotation(robotContainer.photonVision)));

        if (Constants.Teleop.USE_LEGACY_AUTO_ALIGNMENT) {
            driverController.leftBumper().whileTrue(
                    new AutoAlignment(swerve, robotContainer.photonVision, driverController.getHID())
                            .withDesiredAlignmentPosition(AlignmentZone.GenericDesiredAlignmentPosition.LEFT)
            );

            driverController.rightBumper().whileTrue(
                    new AutoAlignment(swerve, robotContainer.photonVision, driverController.getHID())
                            .withDesiredAlignmentPosition(AlignmentZone.GenericDesiredAlignmentPosition.RIGHT)
            );
        } else {
            driverController.leftBumper().whileTrue(
                    new TrajectoryAutoAlignment(
                            robotContainer.swerve,
                            robotContainer.elevator,
                            robotContainer.claw,
                            robotContainer.driverController,
                            robotContainer.photonVision,
                            robotContainer.trajectoryManager
                    ).withDesiredAlignmentSide(AlignmentZone.TrajectoryAlignmentSide.LEFT)
            );

            driverController.rightBumper().whileTrue(
                    new TrajectoryAutoAlignment(
                            robotContainer.swerve,
                            robotContainer.elevator,
                            robotContainer.claw,
                            robotContainer.driverController,
                            robotContainer.photonVision,
                            robotContainer.trajectoryManager
                    ).withDesiredAlignmentSide(AlignmentZone.TrajectoryAlignmentSide.RIGHT)
            );
        }

        // Co Driver
        coDriverController.y().onTrue(
                Commands.runOnce(() -> robotContainer.candleController.setState(SuperstructureStates.CANdleState.YELLOW))
        );
        coDriverController.x().onTrue(
                Commands.runOnce(() -> robotContainer.candleController.setState(SuperstructureStates.CANdleState.PURPLE))
        );

        // ElevatorClawTeleop
        // TODO: make these cleaner, more builder methods have been added since original bindings were made
        //  and some, if not most, bindings can be simplified now

        // Driver Controller
        ElevatorClawTeleop.addMapping(
                driverController.a(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withConditionalElevatorState(
                                SuperstructureStates.ElevatorState.ELEVATOR_CUBE, SuperstructureStates.ElevatorState.ELEVATOR_STANDBY
                        )
                        .withClawState(SuperstructureStates.ClawState.CLAW_INTAKING_CONE)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                driverController.b(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(SuperstructureStates.ElevatorState.ELEVATOR_CUBE, SuperstructureStates.ClawState.CLAW_ANGLE_CUBE)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                driverController.x(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(SuperstructureStates.ElevatorClawStateType.INTAKING)
                        .withClawState(SuperstructureStates.ClawState.CLAW_HOLDING)
                        .waitIfState(SuperstructureStates.ElevatorState.ELEVATOR_DOUBLE_SUB, 0.3)
                        .withElevatorState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                driverController.start(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(SuperstructureStates.ElevatorState.ELEVATOR_TIPPED_CONE, SuperstructureStates.ClawState.TIPPED_CONE)
                        .build()
        );

        // CoDriver Controller
        ElevatorClawTeleop.addMapping(
                coDriverController.a(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(SuperstructureStates.ElevatorClawStateType.SCORING)
                        .withClawState(SuperstructureStates.ClawState.CLAW_OUTTAKE)
                        //TODO: fix wait until state
//                        .waitUntilState(SuperstructureStates.ClawState.CLAW_OUTTAKE)
                        .wait(0.8)
                        .withElevatorClawStates(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY, SuperstructureStates.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.b(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(SuperstructureStates.ElevatorState.ELEVATOR_SINGLE_SUB, SuperstructureStates.ClawState.SINGLE_SUB)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.leftBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT)
                        .withClawState(SuperstructureStates.ClawState.CLAW_SHOOT_HIGH)
                        .waitUntilState(SuperstructureStates.ClawState.CLAW_SHOOT_HIGH)
                        .wait(0.3)
                        .withClawState(SuperstructureStates.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.rightBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfNotInState(SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT)
                        .withClawState(SuperstructureStates.ClawState.CLAW_SHOOT_MID)
                        .waitUntilState(SuperstructureStates.ClawState.CLAW_SHOOT_MID)
                        .wait(0.3)
                        .withClawState(SuperstructureStates.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.leftBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withConditionalNotClawState(SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT, SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT)
                        .build()
        );

        // TODO: fix dropping in low/hybrid nodes
        ElevatorClawTeleop.addMapping(
                coDriverController.rightBumper(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .endIfInState(SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT)
                        .withClawState(SuperstructureStates.ClawState.CLAW_DROP)
                        .wait(0.25)
                        .withClawState(SuperstructureStates.ClawState.CLAW_OUTTAKE_HYBRID)
                        .wait(0.65)
                        .withClawState(SuperstructureStates.ClawState.CLAW_STANDBY)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povUp(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorState(SuperstructureStates.ElevatorState.ELEVATOR_EXTENDED_HIGH)
                        .wait(0.3)
                        .withClawState(SuperstructureStates.ClawState.CLAW_DROP)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povRight(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(SuperstructureStates.ElevatorState.ELEVATOR_EXTENDED_MID, SuperstructureStates.ClawState.CLAW_DROP)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povDown(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorClawStates(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY, SuperstructureStates.ClawState.CLAW_HOLDING)
                        .build()
        );

        ElevatorClawTeleop.addMapping(
                coDriverController.povLeft(),
                new ElevatorClawCommand.Builder(elevator, claw)
                        .withElevatorState(SuperstructureStates.ElevatorState.ELEVATOR_DOUBLE_SUB)
                        .wait(0.1)
                        .withClawState(SuperstructureStates.ClawState.CLAW_INTAKING_CONE)
                        .build()
        );
    }

    public static void clear() {
        ElevatorClawTeleop.removeAllMappings();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }
}
