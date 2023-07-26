package frc.robot.utils.teleop;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class ElevatorClawCommand extends SequentialCommandGroup {
    public ElevatorClawCommand(final Command... commands) {
        super(commands);
    }

    public static class CancelSequentialCommand extends CommandBase {
        private final BooleanSupplier condition;
        private SequentialCommandGroup toCancelCommand;

        public CancelSequentialCommand(final BooleanSupplier condition) {
            this.condition = condition;
        }

        public CancelSequentialCommand() {
            this(() -> true);
        }

        public CancelSequentialCommand(final BooleanSupplier condition, final SequentialCommandGroup toCancelCommand) {
            this.condition = condition;
            setToCancelCommand(toCancelCommand);
        }

        public void setToCancelCommand(final SequentialCommandGroup toCancelCommand) {
            this.toCancelCommand = toCancelCommand;
            addRequirements(toCancelCommand.getRequirements().toArray(Subsystem[]::new));
        }

        public void cancelIfPresentAndCondition() {
            if (toCancelCommand != null && condition.getAsBoolean()) {
                toCancelCommand.cancel();
            }
        }

        @Override
        public void initialize() {
            cancelIfPresentAndCondition();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class Builder {
        private final List<Command> commands;
        private final Elevator elevator;
        private final Claw claw;

        private final Function<Enums.ElevatorState, Boolean> isElevatorState;
        private final Function<Enums.ElevatorStateType, Boolean> isElevatorStateType;
        private final Function<Enums.ClawState, Boolean> isClawState;

        public Builder(final Elevator elevator, final Claw claw) {
            this.elevator = elevator;
            this.claw = claw;

            this.isElevatorState = (state) -> elevator.getDesiredState() == state;
            this.isElevatorStateType = (state) -> elevator.getDesiredState().getElevatorStateType() == state;
            this.isClawState = (state) -> claw.getCurrentState() == state;

            this.commands = new ArrayList<>();
        }

        //TODO: document everything as this can get a bit confusing

        public Builder withStateEndCondition(final Enums.ElevatorState endConditionElevatorState) {
            commands.add(new CancelSequentialCommand(() -> isElevatorState.apply(endConditionElevatorState)));
            return this;
        }

        public Builder withStateEndCondition(final Enums.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> isClawState.apply(endConditionClawState)));
            return this;
        }

        public Builder withStateEndCondition(final Enums.ElevatorStateType endConditionElevatorStateType) {
            commands.add(new CancelSequentialCommand(() -> isElevatorStateType.apply(endConditionElevatorStateType)));
            return this;
        }

        public Builder withNotStateEndCondition(final Enums.ElevatorState endConditionElevatorState) {
            commands.add(new CancelSequentialCommand(() -> !isElevatorState.apply(endConditionElevatorState)));
            return this;
        }

        public Builder withNotStateEndCondition(final Enums.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> !isClawState.apply(endConditionClawState)));
            return this;
        }

        public Builder withNotStateEndCondition(final Enums.ElevatorStateType endConditionElevatorStateType) {
            commands.add(new CancelSequentialCommand(() -> !isElevatorStateType.apply(endConditionElevatorStateType)));
            return this;
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
                    () -> isElevatorState.apply(conditionalElevatorState)
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
                    () -> isClawState.apply(conditionalClawState)
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
                if (isElevatorState.apply(conditionalElevatorState)) {
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
                if (!isElevatorState.apply(conditionalElevatorState)) {
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
                if (isClawState.apply(conditionalClawState)) {
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
                if (!isClawState.apply(conditionalClawState)) {
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

            for (final Command command : commands) {
                if (command instanceof CancelSequentialCommand) {
                    ((CancelSequentialCommand) command).setToCancelCommand(elevatorClawCommand);
                }
            }

            elevatorClawCommand.addCommands(commands.toArray(Command[]::new));
            return elevatorClawCommand;
        }
    }
}
