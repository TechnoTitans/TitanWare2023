package frc.robot.utils.alignment;

import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Enums;
import frc.robot.utils.teleop.ElevatorClawCommand;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public enum SubstationNode {
    LEFT(
            AlignmentZone.SUBSTATION,
            AlignmentZone.GenericDesiredAlignmentPosition.LEFT,
            AlignmentZone.TrajectoryAlignmentSide.LEFT,
            Level.DOUBLE_SUBSTATION
    ),
    RIGHT(
            AlignmentZone.SUBSTATION,
            AlignmentZone.GenericDesiredAlignmentPosition.RIGHT,
            AlignmentZone.TrajectoryAlignmentSide.RIGHT,
            Level.DOUBLE_SUBSTATION
    ),
    SINGLE_SUBSTATION(
            AlignmentZone.SUBSTATION,
            AlignmentZone.GenericDesiredAlignmentPosition.SINGLE_SUB,
            AlignmentZone.TrajectoryAlignmentSide.SINGLE_SUBSTATION,
            Level.SINGLE_SUBSTATION
    );

    public enum Level {
        SINGLE_SUBSTATION(Enums.ElevatorState.ELEVATOR_SINGLE_SUB),
        DOUBLE_SUBSTATION(Enums.ElevatorState.ELEVATOR_DOUBLE_SUB);

        private final Enums.ElevatorState elevatorState;
        Level(final Enums.ElevatorState elevatorState) {
            this.elevatorState = elevatorState;
        }

        public Enums.ElevatorState getElevatorState() {
            return elevatorState;
        }
    }

    private final AlignmentZone alignmentZone;
    private final AlignmentZone.GenericDesiredAlignmentPosition alignmentPosition;
    private final AlignmentZone.TrajectoryAlignmentSide trajectoryAlignmentSide;
    private final Level level;

    public static final Map<AlignmentZone.TrajectoryAlignmentSide, SubstationNode> map = Arrays.stream(values())
            .collect(Collectors.toUnmodifiableMap(
                    SubstationNode::getTrajectoryAlignmentSide,
                    substationNode -> substationNode
            ));

    public static Optional<SubstationNode> getFromTrajectoryAlignmentSide(
            final AlignmentZone.TrajectoryAlignmentSide trajectoryAlignmentSide
    ) {
        return Optional.ofNullable(map.get(trajectoryAlignmentSide));
    }

    SubstationNode(
            final AlignmentZone alignmentZone,
            final AlignmentZone.GenericDesiredAlignmentPosition alignmentPosition,
            final AlignmentZone.TrajectoryAlignmentSide trajectoryAlignmentSide,
            final Level level
    ) {
        this.alignmentZone = alignmentZone;
        this.alignmentPosition = alignmentPosition;
        this.trajectoryAlignmentSide = trajectoryAlignmentSide;
        this.level = level;
    }

    public AlignmentZone getAlignmentZone() {
        return alignmentZone;
    }

    public AlignmentZone.GenericDesiredAlignmentPosition getAlignmentPosition() {
        return alignmentPosition;
    }

    public AlignmentZone.TrajectoryAlignmentSide getTrajectoryAlignmentSide() {
        return trajectoryAlignmentSide;
    }

    public Level getLevel() {
        return level;
    }

    public ElevatorClawCommand buildIntakeSequence(final Elevator elevator, final Claw claw) {
        return new ElevatorClawCommand.Builder(elevator, claw)
                .withElevatorState(Enums.ElevatorState.ELEVATOR_DOUBLE_SUB)
                .wait(0.1)
                .withClawState(Enums.ClawState.CLAW_INTAKING_CUBE)
                .build();
    }

    public ElevatorClawCommand buildRetractSequence(final Elevator elevator, final Claw claw) {
        return new ElevatorClawCommand.Builder(elevator, claw)
                .withClawState(Enums.ClawState.CLAW_INTAKING_CONE)
                .wait(0.1)
                .withClawState(Enums.ClawState.CLAW_HOLDING)
                .wait(0.2)
                .withElevatorState(Enums.ElevatorState.ELEVATOR_STANDBY)
                .build();
    }
}
