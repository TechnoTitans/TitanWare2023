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
        public static final double WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS = 5;

        private final List<Command> commands;
        private final Elevator elevator;
        private final Claw claw;

        private final Function<Enums.ElevatorState, Boolean> isElevatorState;
        private final Function<Enums.ElevatorClawStateType, Boolean> isElevatorClawStateType;
        private final Function<Enums.ClawState, Boolean> isClawState;

        public Builder(final Elevator elevator, final Claw claw) {
            this.elevator = elevator;
            this.claw = claw;

            this.isElevatorState = (state) -> elevator.getCurrentState() == state;
            this.isClawState = (state) -> claw.getCurrentState() == state;
            this.isElevatorClawStateType = (state) ->
                    elevator.getCurrentState().getElevatorStateType() == state
                            || claw.getCurrentState().getElevatorStateType() == state;

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
        public Builder endIfInState(final Enums.ElevatorState endConditionElevatorState) {
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
        public Builder endIfInState(final Enums.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds a {@link Enums.ElevatorClawStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link Enums.ElevatorClawStateType} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorClawStateType the {@link Enums.ElevatorClawStateType} end condition
         * @return this {@link Builder}
         * @see Enums.ElevatorClawStateType
         */
        public Builder endIfInState(final Enums.ElevatorClawStateType endConditionElevatorClawStateType) {
            commands.add(
                    new CancelSequentialCommand(() -> isElevatorClawStateType.apply(endConditionElevatorClawStateType))
            );

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
        public Builder endIfNotInState(final Enums.ElevatorState endConditionElevatorState) {
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
        public Builder endIfNotInState(final Enums.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> !isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds all states except a single {@link Enums.ElevatorClawStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link Enums.ElevatorClawStateType} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorClawStateType the {@link Enums.ElevatorClawStateType} end condition
         * @return this {@link Builder}
         * @see Enums.ElevatorClawStateType
         */
        public Builder endIfNotInState(final Enums.ElevatorClawStateType endConditionElevatorClawStateType) {
            commands.add(
                    new CancelSequentialCommand(() -> !isElevatorClawStateType.apply(endConditionElevatorClawStateType))
            );
            return this;
        }

        /**
         * Adds a wait/delay with a specified amount of time.
         * @param waitSeconds the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see WaitCommand
         */
        public Builder wait(final double waitSeconds) {
            commands.add(Commands.waitSeconds(waitSeconds));
            return this;
        }

        /**
         * Waits until the current {@link frc.robot.utils.Enums.ElevatorState} is equal to the
         * supplied {@link frc.robot.utils.Enums.ElevatorState} or until a timeout
         * @param elevatorState the {@link frc.robot.utils.Enums.ElevatorState} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final Enums.ElevatorState elevatorState, final double timeoutSec) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(() -> isElevatorState.apply(elevatorState))
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link frc.robot.utils.Enums.ElevatorState} is equal to the
         * supplied {@link frc.robot.utils.Enums.ElevatorState} or until a timeout (default amount of time)
         * @param elevatorState the {@link frc.robot.utils.Enums.ElevatorState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final Enums.ElevatorState elevatorState) {
            waitUntilState(elevatorState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link frc.robot.utils.Enums.ClawState} is equal to the
         * supplied {@link frc.robot.utils.Enums.ClawState} or until a timeout
         * @param clawState the {@link frc.robot.utils.Enums.ClawState} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final Enums.ClawState clawState, final double timeoutSec) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(() -> isClawState.apply(clawState))
                    )
            );
            return this;
        }

        /**
         * Waits until the current  {@link frc.robot.utils.Enums.ClawState} is equal to the
         * supplied {@link frc.robot.utils.Enums.ClawState} or until a timeout (default amount of time)
         * @param clawState the {@link frc.robot.utils.Enums.ClawState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final Enums.ClawState clawState) {
            waitUntilState(clawState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link frc.robot.utils.Enums.ElevatorClawStateType} (of the elevator OR claw)
         * is equal to the supplied {@link frc.robot.utils.Enums.ElevatorClawStateType} or until a timeout
         * @param elevatorClawStateType the {@link frc.robot.utils.Enums.ElevatorClawStateType} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(
                final Enums.ElevatorClawStateType elevatorClawStateType,
                final double timeoutSec
        ) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(() -> isElevatorClawStateType.apply(elevatorClawStateType))
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link frc.robot.utils.Enums.ElevatorClawStateType} (of the elevator OR claw)
         * is equal to the supplied {@link frc.robot.utils.Enums.ElevatorClawStateType}
         * or until a timeout (default amount of time)
         * @param elevatorClawStateType the {@link frc.robot.utils.Enums.ElevatorClawStateType} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final Enums.ElevatorClawStateType elevatorClawStateType) {
            waitUntilState(elevatorClawStateType, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link frc.robot.utils.Enums.ElevatorState}
         * and the current {@link frc.robot.utils.Enums.ClawState} are both
         * equal to the supplied {@link frc.robot.utils.Enums.ElevatorState}
         * and {@link frc.robot.utils.Enums.ClawState}, respectively; or until a timeout
         * @param elevatorState the {@link frc.robot.utils.Enums.ElevatorState} to wait for
         * @param clawState the {@link frc.robot.utils.Enums.ClawState} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(
                final Enums.ElevatorState elevatorState,
                final Enums.ClawState clawState,
                final double timeoutSec
        ) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.parallel(
                                    Commands.waitUntil(() -> isElevatorState.apply(elevatorState)),
                                    Commands.waitUntil(() -> isClawState.apply(clawState))
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link frc.robot.utils.Enums.ElevatorState}
         * and the current {@link frc.robot.utils.Enums.ClawState} are both
         * equal to the supplied {@link frc.robot.utils.Enums.ElevatorState}
         * and {@link frc.robot.utils.Enums.ClawState}, respectively; or until a timeout (default amount of time)
         * @param elevatorState the {@link frc.robot.utils.Enums.ElevatorState} to wait for
         * @param clawState the {@link frc.robot.utils.Enums.ClawState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final Enums.ElevatorState elevatorState, final Enums.ClawState clawState) {
            waitUntilState(elevatorState, clawState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
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
        public Builder waitIfState(
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
        public Builder waitIfState(
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
         * @param desiredElevatorState the desired {@link frc.robot.utils.Enums.ElevatorState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         */
        public Builder withConditionalElevatorState(
                final Enums.ElevatorState conditionalElevatorState,
                final Enums.ElevatorState desiredElevatorState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (isElevatorState.apply(conditionalElevatorState)) {
                    elevator.setDesiredState(desiredElevatorState);
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
         * @param desiredElevatorState the desired {@link frc.robot.utils.Enums.ElevatorState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ElevatorState
         */
        public Builder withConditionalNotElevatorState(
                final Enums.ElevatorState conditionalElevatorState,
                final Enums.ElevatorState desiredElevatorState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (!isElevatorState.apply(conditionalElevatorState)) {
                    elevator.setDesiredState(desiredElevatorState);
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
         * @param desiredClawState the desired {@link frc.robot.utils.Enums.ClawState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         */
        public Builder withConditionalClawState(
                final Enums.ClawState conditionalClawState,
                final Enums.ClawState desiredClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (isClawState.apply(conditionalClawState)) {
                    claw.setDesiredState(desiredClawState);
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
         * @param desiredClawState the desired {@link frc.robot.utils.Enums.ClawState}
         * @return this {@link Builder}
         * @see frc.robot.utils.Enums.ClawState
         */
        public Builder withConditionalNotClawState(
                final Enums.ClawState conditionalClawState,
                final Enums.ClawState desiredClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (!isClawState.apply(conditionalClawState)) {
                    claw.setDesiredState(desiredClawState);
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
