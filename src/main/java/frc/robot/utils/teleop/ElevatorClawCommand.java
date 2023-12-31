package frc.robot.utils.teleop;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.SuperstructureStates;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class ElevatorClawCommand extends SequentialCommandGroup {
    public ElevatorClawCommand(final Command... commands) {
        super(commands);
    }

    public static class CancelSequentialCommand extends Command {
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

    @SuppressWarnings({"unused", "UnusedReturnValue"})
    public static class Builder {
        public static final double WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS = 5;

        private final List<Command> commands;
        private final Elevator elevator;
        private final Claw claw;

        private final Function<SuperstructureStates.ElevatorState, Boolean> isElevatorState;
        private final Function<SuperstructureStates.ElevatorState, Boolean> isDesiredElevatorState;

        private final Function<SuperstructureStates.ClawState, Boolean> isClawState;
        private final Function<SuperstructureStates.ClawState, Boolean> isDesiredClawState;

        private final Function<SuperstructureStates.ElevatorClawStateType, Boolean> isElevatorClawStateType;
        private final Function<SuperstructureStates.ElevatorClawStateType, Boolean> isDesiredElevatorClawStateType;

        public Builder(final Elevator elevator, final Claw claw) {
            this.elevator = elevator;
            this.claw = claw;

            this.isElevatorState = elevator::isAtState;
            this.isDesiredElevatorState = (state) -> elevator.getDesiredState() == state;

            this.isClawState = claw::isAtState;
            this.isDesiredClawState = (state) -> claw.getDesiredState() == state;

            this.isElevatorClawStateType = (state) -> {
                final SuperstructureStates.ElevatorState elevatorState = elevator.getCurrentStateWithNullAsTransition();
                final SuperstructureStates.ClawState clawState = claw.getCurrentStateWithNullAsTransition();

                return (elevatorState != null && elevatorState.getElevatorStateType() == state)
                        || (clawState != null && clawState.getElevatorStateType() == state);
            };

            this.isDesiredElevatorClawStateType = (state) -> (
                    (elevator.getDesiredState().getElevatorStateType() == state)
                            || (claw.getDesiredState().getElevatorStateType() == state)
            );

            this.commands = new ArrayList<>();
        }

        /**
         * Adds a {@link SuperstructureStates.ElevatorState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ElevatorState} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorState the {@link SuperstructureStates.ElevatorState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorState
         */
        public Builder endIfInState(final SuperstructureStates.ElevatorState endConditionElevatorState) {
            commands.add(new CancelSequentialCommand(() -> isElevatorState.apply(endConditionElevatorState)));
            return this;
        }

        /**
         * Adds a {@link SuperstructureStates.ClawState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ClawState} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionClawState the {@link SuperstructureStates.ClawState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder endIfInState(final SuperstructureStates.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds a {@link SuperstructureStates.ElevatorClawStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ElevatorClawStateType} (at the current invocation time)
         * matches the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorClawStateType the {@link SuperstructureStates.ElevatorClawStateType} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorClawStateType
         */
        public Builder endIfInState(final SuperstructureStates.ElevatorClawStateType endConditionElevatorClawStateType) {
            commands.add(
                    new CancelSequentialCommand(() -> isElevatorClawStateType.apply(endConditionElevatorClawStateType))
            );

            return this;
        }

        /**
         * Adds all states except a single {@link SuperstructureStates.ElevatorState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ElevatorState} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorState the {@link SuperstructureStates.ElevatorState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorState
         */
        public Builder endIfNotInState(final SuperstructureStates.ElevatorState endConditionElevatorState) {
            commands.add(new CancelSequentialCommand(() -> !isElevatorState.apply(endConditionElevatorState)));
            return this;
        }

        /**
         * Adds all states except a single {@link SuperstructureStates.ClawState} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ClawState} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionClawState the {@link SuperstructureStates.ClawState} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder endIfNotInState(final SuperstructureStates.ClawState endConditionClawState) {
            commands.add(new CancelSequentialCommand(() -> !isClawState.apply(endConditionClawState)));
            return this;
        }

        /**
         * Adds all states except a single {@link SuperstructureStates.ElevatorClawStateType} as an end condition.
         *
         * <p>More formally, invokes the end condition when and only when the current
         * {@link SuperstructureStates.ElevatorClawStateType} (at the current invocation time)
         * does <b>NOT</b> match the supplied end condition state.</p>
         *
         * <p>An end condition is specified by a condition in which all remaining commands (that have yet to run)
         * are cancelled if the condition is met.</p>
         *
         * @param endConditionElevatorClawStateType the {@link SuperstructureStates.ElevatorClawStateType} end condition
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorClawStateType
         */
        public Builder endIfNotInState(final SuperstructureStates.ElevatorClawStateType endConditionElevatorClawStateType) {
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
         * Waits until the current {@link SuperstructureStates.ElevatorState} is equal to the
         * supplied {@link SuperstructureStates.ElevatorState} or until a timeout
         * @param elevatorState the {@link SuperstructureStates.ElevatorState} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ElevatorState elevatorState, final double timeoutSec) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(
                                    () -> isElevatorState.apply(elevatorState)
                                            || !isDesiredElevatorState.apply(elevatorState)
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ElevatorState} is equal to the
         * supplied {@link SuperstructureStates.ElevatorState} or until a timeout (default amount of time)
         * @param elevatorState the {@link SuperstructureStates.ElevatorState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ElevatorState elevatorState) {
            waitUntilState(elevatorState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ClawState} is equal to the
         * supplied {@link SuperstructureStates.ClawState} or until a timeout
         * @param clawState the {@link SuperstructureStates.ClawState} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ClawState clawState, final double timeoutSec) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(
                                    () -> isClawState.apply(clawState) || !isDesiredClawState.apply(clawState)
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current  {@link SuperstructureStates.ClawState} is equal to the
         * supplied {@link SuperstructureStates.ClawState} or until a timeout (default amount of time)
         * @param clawState the {@link SuperstructureStates.ClawState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ClawState clawState) {
            waitUntilState(clawState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ElevatorClawStateType} (of the elevator OR claw)
         * is equal to the supplied {@link SuperstructureStates.ElevatorClawStateType} or until a timeout
         * @param elevatorClawStateType the {@link SuperstructureStates.ElevatorClawStateType} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilState(
                final SuperstructureStates.ElevatorClawStateType elevatorClawStateType,
                final double timeoutSec
        ) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.waitUntil(
                                    () -> isElevatorClawStateType.apply(elevatorClawStateType)
                                            || !isDesiredElevatorClawStateType.apply(elevatorClawStateType)
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ElevatorClawStateType} (of the elevator OR claw)
         * is equal to the supplied {@link SuperstructureStates.ElevatorClawStateType}
         * or until a timeout (default amount of time)
         * @param elevatorClawStateType the {@link SuperstructureStates.ElevatorClawStateType} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilState(final SuperstructureStates.ElevatorClawStateType elevatorClawStateType) {
            waitUntilState(elevatorClawStateType, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ElevatorState}
         * and the current {@link SuperstructureStates.ClawState} are both
         * equal to the supplied {@link SuperstructureStates.ElevatorState}
         * and {@link SuperstructureStates.ClawState}, respectively; or until a timeout
         * @param elevatorState the {@link SuperstructureStates.ElevatorState} to wait for
         * @param clawState the {@link SuperstructureStates.ClawState} to wait for
         * @param timeoutSec the amount of time that can pass until the wait is timed out (seconds)
         * @return this {@link Builder}
         */
        public Builder waitUntilStates(
                final SuperstructureStates.ElevatorState elevatorState,
                final SuperstructureStates.ClawState clawState,
                final double timeoutSec
        ) {
            commands.add(
                    Commands.race(
                            Commands.waitSeconds(timeoutSec),
                            Commands.parallel(
                                    Commands.waitUntil(
                                            () -> isElevatorState.apply(elevatorState)
                                                    || !isDesiredElevatorState.apply(elevatorState)
                                    ),
                                    Commands.waitUntil(
                                            () -> isClawState.apply(clawState)
                                                    || !isDesiredClawState.apply(clawState)
                                    )
                            )
                    )
            );
            return this;
        }

        /**
         * Waits until the current {@link SuperstructureStates.ElevatorState}
         * and the current {@link SuperstructureStates.ClawState} are both
         * equal to the supplied {@link SuperstructureStates.ElevatorState}
         * and {@link SuperstructureStates.ClawState}, respectively; or until a timeout (default amount of time)
         * @param elevatorState the {@link SuperstructureStates.ElevatorState} to wait for
         * @param clawState the {@link SuperstructureStates.ClawState} to wait for
         * @return this {@link Builder}
         */
        public Builder waitUntilStates(final SuperstructureStates.ElevatorState elevatorState, final SuperstructureStates.ClawState clawState) {
            waitUntilStates(elevatorState, clawState, WAIT_FOR_STATE_DEFAULT_TIMEOUT_SECONDS);
            return this;
        }

        /**
         * Adds a conditional wait/delay with a specified amount of time that only runs if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ElevatorState} matches the
         * supplied {@link SuperstructureStates.ElevatorState}.
         *
         * <p>A conditional command is specified by a command that is only ran if certain condition is met.</p>
         * @param conditionalElevatorState the conditional {@link SuperstructureStates.ElevatorState}
         * @param waitSeconds the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorState
         * @see WaitCommand
         */
        public Builder waitIfState(
                final SuperstructureStates.ElevatorState conditionalElevatorState,
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
         * current (or desired, in certain cases) {@link SuperstructureStates.ClawState} matches the
         * supplied {@link SuperstructureStates.ClawState}.
         *
         * <p>A conditional command is specified by a command that is only ran if certain condition is met.</p>
         * @param conditionalClawState the conditional {@link SuperstructureStates.ClawState}
         * @param waitSeconds the amount of time to wait for (seconds)
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         * @see WaitCommand
         */
        public Builder waitIfState(
                final SuperstructureStates.ClawState conditionalClawState,
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
         * Adds a desired {@link SuperstructureStates.ElevatorState}.
         * @param elevatorState the desired {@link SuperstructureStates.ElevatorState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorState
         */
        public Builder withElevatorState(final SuperstructureStates.ElevatorState elevatorState) {
            commands.add(Commands.runOnce(() -> elevator.setDesiredState(elevatorState)));
            return this;
        }

        /**
         * Adds a conditionally desired {@link SuperstructureStates.ElevatorState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ElevatorState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ElevatorState} matches
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalNotElevatorState(SuperstructureStates.ElevatorState, SuperstructureStates.ElevatorState)}</p>
         *
         * @param conditionalElevatorState the conditional {@link SuperstructureStates.ElevatorState}
         * @param desiredElevatorState the desired {@link SuperstructureStates.ElevatorState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorState
         */
        public Builder withConditionalElevatorState(
                final SuperstructureStates.ElevatorState conditionalElevatorState,
                final SuperstructureStates.ElevatorState desiredElevatorState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (isElevatorState.apply(conditionalElevatorState)) {
                    elevator.setDesiredState(desiredElevatorState);
                }
            }));
            return this;
        }

        /**
         * Adds a conditionally-inverted desired {@link SuperstructureStates.ElevatorState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ElevatorState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ElevatorState} does <b>NOT</b> match
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalElevatorState(SuperstructureStates.ElevatorState, SuperstructureStates.ElevatorState)}</p>
         *
         * @param conditionalElevatorState the conditional {@link SuperstructureStates.ElevatorState}
         * @param desiredElevatorState the desired {@link SuperstructureStates.ElevatorState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ElevatorState
         */
        public Builder withConditionalNotElevatorState(
                final SuperstructureStates.ElevatorState conditionalElevatorState,
                final SuperstructureStates.ElevatorState desiredElevatorState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (!isElevatorState.apply(conditionalElevatorState)) {
                    elevator.setDesiredState(desiredElevatorState);
                }
            }));
            return this;
        }

        /**
         * Adds a desired {@link SuperstructureStates.ClawState}.
         * @param clawState the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder withClawState(final SuperstructureStates.ClawState clawState) {
            commands.add(Commands.runOnce(() -> claw.setDesiredState(clawState)));
            return this;
        }

        /**
         * Adds a conditionally desired {@link SuperstructureStates.ClawState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ClawState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ClawState} matches
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalNotClawState(SuperstructureStates.ClawState, SuperstructureStates.ClawState)}</p>
         *
         * @param conditionalClawState the conditional {@link SuperstructureStates.ClawState}
         * @param desiredClawState the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder withConditionalClawState(
                final SuperstructureStates.ClawState conditionalClawState,
                final SuperstructureStates.ClawState desiredClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (isClawState.apply(conditionalClawState)) {
                    claw.setDesiredState(desiredClawState);
                }
            }));
            return this;
        }

        /**
         * Adds a conditionally-inverted desired {@link SuperstructureStates.ClawState}.
         *
         * <p>More formally, sets the desired {@link SuperstructureStates.ClawState} if and only if the
         * current (or desired, in certain cases) {@link SuperstructureStates.ClawState} does <b>NOT</b> match
         * the conditional state</p>
         *
         * <p>This is the logical inverse/negation of
         * {@link Builder#withConditionalClawState(SuperstructureStates.ClawState, SuperstructureStates.ClawState)}</p>
         *
         * @param conditionalClawState the conditional {@link SuperstructureStates.ClawState}
         * @param desiredClawState the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see SuperstructureStates.ClawState
         */
        public Builder withConditionalNotClawState(
                final SuperstructureStates.ClawState conditionalClawState,
                final SuperstructureStates.ClawState desiredClawState
        ) {
            commands.add(Commands.runOnce(() -> {
                if (!isClawState.apply(conditionalClawState)) {
                    claw.setDesiredState(desiredClawState);
                }
            }));
            return this;
        }

        /**
         * Adds a desired {@link SuperstructureStates.ElevatorState} and {@link SuperstructureStates.ClawState}.
         * @param elevatorState the desired {@link SuperstructureStates.ElevatorState}
         * @param clawState the desired {@link SuperstructureStates.ClawState}
         * @return this {@link Builder}
         * @see Builder#withElevatorState(SuperstructureStates.ElevatorState)
         * @see Builder#withClawState(SuperstructureStates.ClawState)
         * @see SuperstructureStates.ElevatorState
         * @see SuperstructureStates.ClawState
         */
        public Builder withElevatorClawStates(
                final SuperstructureStates.ElevatorState elevatorState,
                final SuperstructureStates.ClawState clawState
        ) {
            commands.add(Commands.runOnce(() -> {
                elevator.setDesiredState(elevatorState);
                claw.setDesiredState(clawState);
            }));

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
