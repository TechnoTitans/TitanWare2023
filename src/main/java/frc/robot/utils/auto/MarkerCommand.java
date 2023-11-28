package frc.robot.utils.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.TrajectoryFollower;
import frc.robot.constants.Constants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.alignment.GridNode;
import frc.robot.utils.teleop.ElevatorClawCommand;

import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Collectors;

public enum MarkerCommand {
    CLAW(
            "claw",
            List.of(ArgumentChecker.enumChecker(SuperstructureStates.ClawState.class)),
            ((followerContext, args) ->
                    new ElevatorClawCommand.Builder(followerContext.getElevator(), followerContext.getClaw())
                            .withClawState(
                                    SuperstructureStates.ClawState.valueOf(args.get(0))
                            )
                            .build()
            )
    ),
    ELEVATOR(
            "elevator",
            List.of(ArgumentChecker.enumChecker(SuperstructureStates.ElevatorState.class)),
            ((followerContext, args) ->
                    new ElevatorClawCommand.Builder(followerContext.getElevator(), followerContext.getClaw())
                            .withElevatorState(
                                    SuperstructureStates.ElevatorState.valueOf(args.get(0))
                            )
                            .build()
            )
    ),
    SCORE(
            "score",
            List.of(ArgumentChecker.enumChecker(GridNode.Level.class)),
            (((followerContext, args) ->
                    Commands.sequence(
                            Commands.runOnce(() -> followerContext.setPaused(true)),
                            GridNode.buildScoringSequence(
                                    followerContext.getElevator(),
                                    followerContext.getClaw(),
                                    GridNode.Level.valueOf(args.get(0))
                            ),
                            Commands.waitSeconds(0.6),
                            Commands.runOnce(() -> followerContext.setPaused(false))
                    )
            ))
    ),
    INTAKE(
            "intake",
            List.of(ArgumentChecker.enumChecker(SuperstructureStates.IntakeMode.class)),
            ((followerContext, args) -> {
                final SuperstructureStates.IntakeMode intakeMode =
                        SuperstructureStates.IntakeMode.valueOf(args.get(0));

                return new ElevatorClawCommand.Builder(followerContext.getElevator(), followerContext.getClaw())
                        .withElevatorClawStates(
                                intakeMode.getElevatorState(),
                                intakeMode.getClawState()
                        )
                        .build();
            })
    ),
    PICKUP(
            "pickup",
            List.of(),
            ((followerContext, args) ->
                    new ElevatorClawCommand.Builder(followerContext.getElevator(), followerContext.getClaw())
                            .withConditionalClawState(
                                    SuperstructureStates.ClawState.CLAW_INTAKING_CUBE,
                                    SuperstructureStates.ClawState.CLAW_INTAKING_CONE
                            )
                            .waitUntilState(
                                    SuperstructureStates.ClawState.CLAW_INTAKING_CONE,
                                    0.7
                            )
                            .withElevatorClawStates(
                                    SuperstructureStates.ElevatorState.ELEVATOR_STANDBY,
                                    SuperstructureStates.ClawState.CLAW_HOLDING
                            )
                            .build()
            )
    ),
    SHOOT(
            "shoot",
            List.of(ArgumentChecker.enumChecker(GridNode.Level.class)),
            (((followerContext, args) ->
                    Commands.sequence(
                            Commands.runOnce(() -> followerContext.setPaused(true)),
                            GridNode.buildShootingSequence(
                                    followerContext.getElevator(),
                                    followerContext.getClaw(),
                                    GridNode.Level.valueOf(args.get(0))
                            ),
                            Commands.waitSeconds(0.1),
                            Commands.runOnce(() -> followerContext.setPaused(false))
                    )
            ))
    ),
    WAIT(
            "wait",
            List.of(ArgumentChecker.doubleChecker()),
            ((followerContext, args) -> Commands.waitSeconds(Double.parseDouble(args.get(0))))
    ),
    // TODO: this messes with the PathPlanner computed time, causing it not to work properly
    //  either fix it or remove this
    HOLD_VELOCITY(
            "holdvel",
            List.of(ArgumentChecker.doubleChecker()),
            ((followerContext, args) -> Commands.runOnce(
                    () -> followerContext.setModuleMaxSpeed(Double.parseDouble(args.get(0)))
            ))
    ),
    WHEEL_X(
            "wheelx",
            List.of(ArgumentChecker.booleanChecker()),
            ((followerContext, args) -> Commands.runOnce(
                    () -> followerContext.setWheelX(Boolean.parseBoolean(args.get(0)))
            ))
    ),
    AUTO_BALANCE(
            "autobalance",
            List.of(),
            (((followerContext, args) -> new AutoBalance(followerContext.getSwerve())))
    ),
    DT_PAUSE(
            "dtpause",
            List.of(ArgumentChecker.booleanChecker()),
            ((followerContext, args) -> Commands.runOnce(
                    () -> followerContext.setPaused(Boolean.parseBoolean(args.get(0)))
            ))
    );

    public static final String ARGS_DELIMITER = ":";
    public static final String COMMAND_DELIMITER = ";";

    private final String name;
    private final List<ArgumentChecker<String>> argumentCheckers;
    private final int argCount;
    private final BiFunction<TrajectoryFollower.FollowerContext, List<String>, Command> commandFunction;

    private static final Map<String, MarkerCommand> markerCommandMap = Arrays.stream(values())
                    .collect(Collectors.toUnmodifiableMap(
                            MarkerCommand::getName,
                            markerCommand -> markerCommand
                    ));

    MarkerCommand(
            final String name,
            final List<ArgumentChecker<String>> argumentCheckers,
            final BiFunction<TrajectoryFollower.FollowerContext, List<String>, Command> commandFunction
    ) {
        this.name = name.toUpperCase();
        this.argumentCheckers = argumentCheckers;
        this.argCount = argumentCheckers.size();
        this.commandFunction = commandFunction;
    }

    public static Command get(final String stringCommand, final TrajectoryFollower.FollowerContext followerContext) {
        final String[] args = stringCommand.split(ARGS_DELIMITER);
        if (args.length < 1) {
            return Commands.none();
        }

        final MarkerCommand markerCommand = markerCommandMap.get(args[0]);
        return markerCommand != null
                ? markerCommand.command(followerContext, Arrays.stream(args).toList())
                : Commands.none();
    }

    public String getName() {
        return name;
    }

    private void reportArgumentMismatch(final ArgumentMismatchException mismatchException) {
        final boolean isCompetition = Constants.CURRENT_COMPETITION_TYPE == Constants.CompetitionType.COMPETITION;

        DriverStation.reportError(mismatchException.toString(), mismatchException.getStackTrace());
        if (!isCompetition) {
            throw mismatchException;
        }
    }

    public Command command(final TrajectoryFollower.FollowerContext followerContext, final List<String> argsWithName) {
        final List<String> args = argsWithName.subList(1, argsWithName.size());
        final int providedArgCount = args.size();
        if (providedArgCount != argCount) {
            reportArgumentMismatch(new ArgumentMismatchException(argCount, providedArgCount));
            return Commands.none();
        }

        final ListIterator<ArgumentChecker<String>> argsCheckerIterator = argumentCheckers.listIterator();
        final ListIterator<String> argsIterator = args.listIterator();

        while (argsCheckerIterator.hasNext() && argsIterator.hasNext()) {
            final ArgumentChecker<String> argumentChecker = argsCheckerIterator.next();
            final String arg = argsIterator.next();

            final boolean argIsOk = argumentChecker.check(arg);
            if (!argIsOk) {
                reportArgumentMismatch(new ArgumentMismatchException(argumentChecker, arg));
                return Commands.none();
            }
        }

        return commandFunction.apply(followerContext, argsWithName.subList(1, argsWithName.size()));
    }

    public static class ArgumentMismatchException extends RuntimeException {
        public ArgumentMismatchException(final int expectedCount, final int gotCount) {
            super(String.format("MarkerCommand expected %d arguments; instead got %d arguments!", expectedCount, gotCount));
        }

        public ArgumentMismatchException(final ArgumentChecker<?> argumentChecker, final String gotArg) {
            super(String.format("MarkerCommand %s reported incorrect arg: %s", argumentChecker, gotArg));
        }
    }

    public static class ArgumentChecker<T> {
        private final Function<T, Boolean> checkFunction;

        public ArgumentChecker(final Function<T, Boolean> checkFunction) {
            this.checkFunction = checkFunction;
        }

        public static <T> ArgumentChecker<T> allowAll() {
            return new ArgumentChecker<>(t -> true);
        }

        public static <T extends Enum<T>> ArgumentChecker<String> enumChecker(final Class<T> tClass) {
            return new ArgumentChecker<>(name -> {
                try {
                    Enum.valueOf(tClass, name);
                    return true;
                } catch (final IllegalArgumentException illegalArgumentException) {
                    return false;
                }
            });
        }

        public static ArgumentChecker<String> doubleChecker() {
            return new ArgumentChecker<>(str -> {
                try {
                    Double.parseDouble(str);
                    return true;
                } catch (final NumberFormatException numberFormatException) {
                    return false;
                }
            });
        }

        public static ArgumentChecker<String> booleanChecker() {
            // all strings can be parsed as a boolean, as all strings that aren't true are parsed as false
            // thus, we can just allowAll here
            return allowAll();
        }

        public boolean check(final T input) {
            return checkFunction.apply(input);
        }
    }
}
