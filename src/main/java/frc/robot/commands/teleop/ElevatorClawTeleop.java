package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.teleop.ElevatorClawCommand;

import java.util.HashMap;
import java.util.Map;

public class ElevatorClawTeleop extends Command {
    private static final Map<Trigger, ElevatorClawCommand> controllerMappings = new HashMap<>();

    private final Elevator elevator;
    private final Claw claw;

    public ElevatorClawTeleop(
            final Elevator elevator,
            final Claw claw
    ) {
        this.elevator = elevator;
        this.claw = claw;

        addRequirements(elevator, claw);
    }

    public static void addMapping(
            final Trigger buttonPressedTrigger,
            final ElevatorClawCommand elevatorClawCommand
    ) {
        if (controllerMappings.containsKey(buttonPressedTrigger)) {
            throw new RuntimeException("Attempted to map the same ButtonPressedSupplier twice!");
        }

        controllerMappings.put(buttonPressedTrigger, elevatorClawCommand);
    }

    public static void removeMapping(final Trigger buttonPressedTrigger) {
        controllerMappings.remove(buttonPressedTrigger);
    }

    public static void removeAllMappings() {
        controllerMappings.clear();
    }

    @Override
    public void initialize() {
        final SuperstructureStates.ElevatorState elevatorState = elevator.getDesiredState();
        if (elevatorState != SuperstructureStates.ElevatorState.ELEVATOR_RESET
                && elevatorState != SuperstructureStates.ElevatorState.ELEVATOR_STANDBY
        ) {
            elevator.setDesiredState(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY);
        }

        final SuperstructureStates.ClawState clawState = claw.getDesiredState();
        if (clawState != SuperstructureStates.ClawState.CLAW_STANDBY) {
            claw.setDesiredState(SuperstructureStates.ClawState.CLAW_STANDBY);
        }

        for (final Map.Entry<Trigger, ElevatorClawCommand> mapping : controllerMappings.entrySet()) {
            final Trigger mappedTrigger = mapping.getKey();
            final ElevatorClawCommand mappedCommand = mapping.getValue();

            mappedTrigger.onTrue(Commands.runOnce(() -> {
                if (!CommandScheduler.getInstance().isScheduled(mappedCommand)) {
                    mappedCommand.schedule();
                }
            }));
        }
    }
}
