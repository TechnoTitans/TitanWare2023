package frc.robot.utils.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.constants.Constants;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.alignment.GridNode;
import frc.robot.utils.teleop.ElevatorClawCommand;

import java.util.*;
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
                    Commands.parallel(
                            Commands.idle(followerContext.getSwerve()),
                            Commands.sequence(
                                GridNode.buildScoringSequence(
                                        followerContext.getElevator(),
                                        followerContext.getClaw(),
                                        GridNode.Level.valueOf(args.get(0))
                                ),
                                Commands.waitSeconds(0.6)
                        )
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
                            GridNode.buildShootingSequence(
                                    followerContext.getElevator(),
                                    followerContext.getClaw(),
                                    GridNode.Level.valueOf(args.get(0))
                            ),
                            Commands.waitSeconds(0.1)
                    )
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
    );

    public static final String ARGS_DELIMITER = ":";

    private final String name;
    private final List<ArgumentChecker<String>> argumentCheckers;
    private final int argCount;
    private final BiFunction<TrajectoryManager.FollowerContext, List<String>, Command> commandFunction;

    private static final Map<String, MarkerCommand> markerCommandMap = Arrays.stream(values())
            .collect(Collectors.toUnmodifiableMap(
                    MarkerCommand::getName,
                    markerCommand -> markerCommand
            ));

    MarkerCommand(
            final String name,
            final List<ArgumentChecker<String>> argumentCheckers,
            final BiFunction<TrajectoryManager.FollowerContext, List<String>, Command> commandFunction
    ) {
        this.name = name.toUpperCase();
        this.argumentCheckers = argumentCheckers;
        this.argCount = argumentCheckers.size();
        this.commandFunction = commandFunction;
    }

    public static void setupPathPlannerNamedCommands(final TrajectoryManager.FollowerContext followerContext) {
        for (final Map.Entry<String, MarkerCommand> markerCommandEntry : markerCommandMap.entrySet()) {
            final String markerCommandName = markerCommandEntry.getKey();
            final MarkerCommand markerCommand = markerCommandEntry.getValue();

            final Set<List<String>> argumentCombinations =
                    ArgumentChecker.cartesianProduct(markerCommand.argumentCheckers);

            for (final List<String> args : argumentCombinations) {
                final StringBuilder builder = new StringBuilder(markerCommandName);
                for (final String arg : args) {
                    builder.append(ARGS_DELIMITER);
                    builder.append(arg);
                }

                final String commandName = builder.toString().toLowerCase();
                final Command command = markerCommand.command(followerContext, args);
                NamedCommands.registerCommand(commandName, command);
            }
        }
    }

    public static Command get(final String stringCommand, final TrajectoryManager.FollowerContext followerContext) {
        final String[] args = stringCommand.split(ARGS_DELIMITER);
        if (args.length < 1) {
            return Commands.none();
        }

        final MarkerCommand markerCommand = markerCommandMap.get(args[0]);
        return markerCommand != null
                ? markerCommand.command(followerContext, Arrays.stream(args).toList().subList(1, args.length))
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

    public Command command(final TrajectoryManager.FollowerContext followerContext, final List<String> args) {
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

        return commandFunction.apply(followerContext, args);
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
        private final Set<T> options;

        public ArgumentChecker(
                final Function<T, Boolean> checkFunction,
                final Set<T> options
        ) {
            this.checkFunction = checkFunction;
            this.options = options;
        }

        public static <T> Set<List<T>> cartesianProduct(final List<ArgumentChecker<T>> argumentCheckers) {
            final List<Set<T>> allOptions = argumentCheckers.stream()
                    .map(ArgumentChecker::getOptions)
                    .toList();

            return cartesianProduct(0, allOptions);
        }

        private static <T> Set<List<T>> cartesianProduct(final int index, final List<Set<T>> sets) {
            final Set<List<T>> ret = new HashSet<>();
            if (index == sets.size()) {
                ret.add(new ArrayList<>());
            } else {
                for (final T t : sets.get(index)) {
                    for (List<T> tList : cartesianProduct(index + 1, sets)) {
                        tList.add(t);
                        ret.add(tList);
                    }
                }
            }

            return ret;
        }

        public static <T extends Enum<T>> ArgumentChecker<String> enumChecker(final Class<T> tClass) {
            return new ArgumentChecker<>(
                    name -> {
                        try {
                            Enum.valueOf(tClass, name);
                            return true;
                        } catch (final IllegalArgumentException illegalArgumentException) {
                            return false;
                        }
                    },
                    Arrays.stream(tClass.getEnumConstants())
                            .map(Enum::name)
                            .collect(Collectors.toSet())
            );
        }

        public static ArgumentChecker<String> booleanChecker() {
            // all strings can be parsed as a boolean, as all strings that aren't true are parsed as false
            // thus, we can just allowAll here
            final Set<String> boolOptions = Set.of("TRUE", "FALSE");
            return new ArgumentChecker<>(
                    boolOptions::contains,
                    boolOptions
            );
        }

        public boolean check(final T input) {
            return checkFunction.apply(input);
        }

        public Set<T> getOptions() {
            return options;
        }
    }
}