package frc.robot.utils.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;

import java.util.ArrayList;
import java.util.List;

public class ElevatorClawCommand extends SequentialCommandGroup {
    public static class Builder {
        private final List<Command> commands;
        private final Elevator elevator;
        private final Claw claw;

        public Builder(final Elevator elevator, final Claw claw) {
            this.elevator = elevator;
            this.claw = claw;

            this.commands = new ArrayList<>();
        }

        public Builder withWait(final double waitSeconds) {
            commands.add(Commands.waitSeconds(waitSeconds));
            return this;
        }

        public Builder withConditionalWait(
                final Enums.ElevatorState conditionalElevatorState,
                final double waitSeconds
        ) {
            commands.add(Commands.either(
                    Commands.waitSeconds(waitSeconds),
                    Commands.none(),
                    () -> elevator.getDesiredState() == conditionalElevatorState
            ));
            return this;
        }

        public Builder withConditionalWait(
                final Enums.ClawState conditionalClawState,
                final double waitSeconds
        ) {
            commands.add(Commands.either(
                    Commands.waitSeconds(waitSeconds),
                    Commands.none(),
                    () -> claw.getDesiredState() == conditionalClawState
            ));
            return this;
        }

        public Builder withElevatorState(final Enums.ElevatorState elevatorState) {
            commands.add(Commands.runOnce(() -> elevator.setDesiredState(elevatorState)));
            return this;
        }

        public Builder withConditionalElevatorState(
                final Enums.ElevatorState conditionalElevatorState,
                final Enums.ElevatorState onConditionalElevatorState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (elevator.getDesiredState() == conditionalElevatorState) {
                    elevator.setDesiredState(onConditionalElevatorState);
                }
            }));
            return this;
        }

        public Builder withConditionalNotElevatorState(
                final Enums.ElevatorState conditionalElevatorState,
                final Enums.ElevatorState onConditionalElevatorState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (elevator.getDesiredState() != conditionalElevatorState) {
                    elevator.setDesiredState(onConditionalElevatorState);
                }
            }));
            return this;
        }

        public Builder withClawState(final Enums.ClawState clawState) {
            commands.add(Commands.runOnce(() -> claw.setDesiredState(clawState)));
            return this;
        }

        public Builder withConditionalClawState(
                final Enums.ClawState conditionalClawState,
                final Enums.ClawState onConditionalClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (claw.getCurrentState() == conditionalClawState) {
                    claw.setDesiredState(onConditionalClawState);
                }
            }));
            return this;
        }

        public Builder withConditionalNotClawState(
                final Enums.ClawState conditionalClawState,
                final Enums.ClawState onConditionalClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (claw.getCurrentState() != conditionalClawState) {
                    claw.setDesiredState(onConditionalClawState);
                }
            }));
            return this;
        }

        public Builder withElevatorClawStates(
                final Enums.ElevatorState elevatorState,
                final Enums.ClawState clawState
        ) {
            withElevatorState(elevatorState);
            withClawState(clawState);
            return this;
        }

        public ElevatorClawCommand build() {
            final ElevatorClawCommand elevatorClawCommand = new ElevatorClawCommand();
            elevatorClawCommand.addCommands(commands.toArray(Command[]::new));
            return elevatorClawCommand;
        }
    }
}
