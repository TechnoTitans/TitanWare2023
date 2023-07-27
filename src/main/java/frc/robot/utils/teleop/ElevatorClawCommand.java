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

        @SuppressWarnings("unused")
        public CancelSequentialCommand() {
            this(() -> true);
        }

        @SuppressWarnings("unused")
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

    @SuppressWarnings("unused")
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

        /**
         * Adds a {@link frc.robot.utils.Enums.ElevatorState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link frc.robot.utils.Enums.ElevatorState} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorState the {@link frc.robot.utils.Enums.ElevatorState} end condition
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         */
        public Builder withStateEndCondition(final Enums.ElevatorState endConditionElevatorState) {
            commands.add(new CancelSequentialCommand(() -> isElevatorState.apply(endConditionElevatorState)));
            return this;
        }

        /**
         * Adds a {@link frc.robot.utils.Enums.ClawState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link frc.robot.utils.Enums.ClawState} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionClawState the {@link frc.robot.utils.Enums.ClawState} end condition
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         */
        public Builder withStateEndCondition(final Enums.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds a {@link frc.robot.utils.Enums.ElevatorStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link frc.robot.utils.Enums.ElevatorStateType} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorStateType the {@link frc.robot.utils.Enums.ElevatorStateType} end condition
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorStateType
         */
        public Builder withStateEndCondition(final Enums.ElevatorStateType endConditionElevatorStateType) {
            commands.add(new CancelSequentialCommand(() -> isElevatorStateType.apply(endConditionElevatorStateType)));
            return this;
        }

        /**
         * Adds all states except a single {@link frc.robot.utils.Enums.ElevatorState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link frc.robot.utils.Enums.ElevatorState} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorState the {@link frc.robot.utils.Enums.ElevatorState} end condition
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         */
        public Builder withNotStateEndCondition(final Enums.ElevatorState endConditionElevatorState) {
            commands.add(new CancelSequentialCommand(() -> !isElevatorState.apply(endConditionElevatorState)));
            return this;
        }

        /**
         * Adds all states except a single {@link frc.robot.utils.Enums.ClawState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link frc.robot.utils.Enums.ClawState} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionClawState the {@link frc.robot.utils.Enums.ClawState} end condition
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         */
        public Builder withNotStateEndCondition(final Enums.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> !isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds all states except a single {@link frc.robot.utils.Enums.ElevatorStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link frc.robot.utils.Enums.ElevatorStateType} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorStateType the {@link frc.robot.utils.Enums.ElevatorStateType} end condition
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorStateType
         */
        public Builder withNotStateEndCondition(final Enums.ElevatorStateType endConditionElevatorStateType) {
            commands.add(new CancelSequentialCommand(() -> !isElevatorStateType.apply(endConditionElevatorStateType)));
            return this;
        }

        /**
         * Adds a wait/delay with a specified amount of time.
         * @param waitSeconds the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see WaitCommand
         */
        public Builder withWait(final double waitSeconds) {
            commands.add(Commands.waitSeconds(waitSeconds));
            return this;
        }

        /**
         * Adds a conditional wait/delay with a specified amount of time that only runs if the
         * current (or desired, in certain cases) {@link frc.robot.utils.Enums.ElevatorState} matches the
         * supplied {@link frc.robot.utils.Enums.ElevatorState}.
         *
         * <p>A conditional command is specified by a command that is only ran if certain condition is met.</p>
         * @param conditionalElevatorState the conditional {@link frc.robot.utils.Enums.ElevatorState}
         * @param waitSeconds the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         * @see WaitCommand
         */
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

        /**
         * Adds a conditional wait/delay with a specified amount of time that only runs if the
         * current (or desired, in certain cases) {@link frc.robot.utils.Enums.ClawState} matches the
         * supplied {@link frc.robot.utils.Enums.ClawState}.
         *
         * <p>A conditional command is specified by a command that is only ran if certain condition is met.</p>
         * @param conditionalClawState the conditional {@link frc.robot.utils.Enums.ClawState}
         * @param waitSeconds the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         * @see WaitCommand
         */
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

        /**
         * Adds a desired {@link frc.robot.utils.Enums.ElevatorState}.
         * @param elevatorState the desired {@link frc.robot.utils.Enums.ElevatorState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         */
        public Builder withElevatorState(final Enums.ElevatorState elevatorState) {
            commands.add(Commands.runOnce(() -> elevator.setDesiredState(elevatorState)));
            return this;
        }

        /**
         * Adds a conditionally desired {@link frc.robot.utils.Enums.ElevatorState}.
         *
         * <p>More formally, sets the desired {@link frc.robot.utils.Enums.ElevatorState} if and only if the
         * current (or desired, in certain cases) {@link frc.robot.utils.Enums.ElevatorState} matches
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalNotElevatorState(Enums.ElevatorState, Enums.ElevatorState)}</p>
         *
         * @param conditionalElevatorState the conditional {@link frc.robot.utils.Enums.ElevatorState}
         * @param onConditionalElevatorState the desired {@link frc.robot.utils.Enums.ElevatorState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         */
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

        /**
         * Adds a conditionally-inverted desired {@link frc.robot.utils.Enums.ElevatorState}.
         *
         * <p>More formally, sets the desired {@link frc.robot.utils.Enums.ElevatorState} if and only if the
         * current (or desired, in certain cases) {@link frc.robot.utils.Enums.ElevatorState} does <b>NOT</b> match
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalElevatorState(Enums.ElevatorState, Enums.ElevatorState)}</p>
         *
         * @param conditionalElevatorState the conditional {@link frc.robot.utils.Enums.ElevatorState}
         * @param onConditionalElevatorState the desired {@link frc.robot.utils.Enums.ElevatorState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         */
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

        /**
         * Adds a desired {@link frc.robot.utils.Enums.ClawState}.
         * @param clawState the desired {@link frc.robot.utils.Enums.ClawState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         */
        public Builder withClawState(final Enums.ClawState clawState) {
            commands.add(Commands.runOnce(() -> claw.setDesiredState(clawState)));
            return this;
        }

        /**
         * Adds a conditionally desired {@link frc.robot.utils.Enums.ClawState}.
         *
         * <p>More formally, sets the desired {@link frc.robot.utils.Enums.ClawState} if and only if the
         * current (or desired, in certain cases) {@link frc.robot.utils.Enums.ClawState} matches
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalNotClawState(Enums.ClawState, Enums.ClawState)}</p>
         *
         * @param conditionalClawState the conditional {@link frc.robot.utils.Enums.ClawState}
         * @param onConditionalClawState the desired {@link frc.robot.utils.Enums.ClawState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         */
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

        /**
         * Adds a conditionally-inverted desired {@link frc.robot.utils.Enums.ClawState}.
         *
         * <p>More formally, sets the desired {@link frc.robot.utils.Enums.ClawState} if and only if the
         * current (or desired, in certain cases) {@link frc.robot.utils.Enums.ClawState} does <b>NOT</b> match
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalClawState(Enums.ClawState, Enums.ClawState)}</p>
         *
         * @param conditionalClawState the conditional {@link frc.robot.utils.Enums.ClawState}
         * @param onConditionalClawState the desired {@link frc.robot.utils.Enums.ClawState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         */
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

        /**
         * Adds a desired {@link frc.robot.utils.Enums.ElevatorState} and {@link frc.robot.utils.Enums.ClawState}.
         * @param elevatorState the desired {@link frc.robot.utils.Enums.ElevatorState}
         * @param clawState the desired {@link frc.robot.utils.Enums.ClawState}
         * @return this {@link Builder}
         * @see Builder#withElevatorState(Enums.ElevatorState)
         * @see Builder#withClawState(Enums.ClawState)
         * @see frc.robot.utils.Enums.ElevatorState
         * @see frc.robot.utils.Enums.ClawState
         */
        public Builder withElevatorClawStates(
                final Enums.ElevatorState elevatorState,
                final Enums.ClawState clawState
        ) {
            withElevatorState(elevatorState);
            withClawState(clawState);
            return this;
        }

        /**
         * Builds this {@link Builder}, producing a {@link ElevatorClawCommand}
         * @return the produced {@link ElevatorClawCommand}
         */
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
