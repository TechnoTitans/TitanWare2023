package frc.robot.utils.alignment;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.SuperstructureStates;
import frc.robot.utils.teleop.ElevatorClawCommand;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import static frc.robot.utils.alignment.AlignmentZone.GenericDesiredAlignmentPosition;

public enum GridNode {
    LEFT_LEFT_HIGH(NTGridNode.LEFT_LEFT_HIGH, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.LEFT, Level.HIGH),
    LEFT_LEFT_MID(NTGridNode.LEFT_LEFT_MID, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.LEFT, Level.MID),
    LEFT_LEFT_LOW(NTGridNode.LEFT_LEFT_LOW, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.LEFT, Level.LOW),

    LEFT_CENTER_HIGH(NTGridNode.LEFT_CENTER_HIGH, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.CENTER, Level.HIGH),
    LEFT_CENTER_MID(NTGridNode.LEFT_CENTER_MID, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.CENTER, Level.MID),
    LEFT_CENTER_LOW(NTGridNode.LEFT_CENTER_LOW, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.CENTER, Level.LOW),

    LEFT_RIGHT_HIGH(NTGridNode.LEFT_RIGHT_HIGH, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.RIGHT, Level.HIGH),
    LEFT_RIGHT_MID(NTGridNode.LEFT_RIGHT_MID, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.RIGHT, Level.MID),
    LEFT_RIGHT_LOW(NTGridNode.LEFT_RIGHT_LOW, AlignmentZone.LEFT, GenericDesiredAlignmentPosition.RIGHT, Level.LOW),

    //No 9, only 9 nodes per row on grid

    CENTER_LEFT_HIGH(NTGridNode.CENTER_LEFT_HIGH, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.LEFT, Level.HIGH),
    CENTER_LEFT_MID(NTGridNode.CENTER_LEFT_MID, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.LEFT, Level.MID),
    CENTER_LEFT_LOW(NTGridNode.CENTER_LEFT_LOW, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.LEFT, Level.LOW),

    CENTER_CENTER_HIGH(NTGridNode.CENTER_CENTER_HIGH, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.CENTER, Level.HIGH),
    CENTER_CENTER_MID(NTGridNode.CENTER_CENTER_MID, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.CENTER, Level.MID),
    CENTER_CENTER_LOW(NTGridNode.CENTER_CENTER_LOW, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.CENTER, Level.LOW),

    CENTER_RIGHT_HIGH(NTGridNode.CENTER_RIGHT_HIGH, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.RIGHT, Level.HIGH),
    CENTER_RIGHT_MID(NTGridNode.CENTER_RIGHT_MID, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.RIGHT, Level.MID),
    CENTER_RIGHT_LOW(NTGridNode.CENTER_RIGHT_LOW, AlignmentZone.CENTER, GenericDesiredAlignmentPosition.RIGHT, Level.LOW),

    //No 19

    RIGHT_LEFT_HIGH(NTGridNode.RIGHT_LEFT_HIGH, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.LEFT, Level.HIGH),
    RIGHT_LEFT_MID(NTGridNode.RIGHT_LEFT_MID, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.LEFT, Level.MID),
    RIGHT_LEFT_LOW(NTGridNode.RIGHT_LEFT_LOW, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.LEFT, Level.LOW),

    RIGHT_CENTER_HIGH(NTGridNode.RIGHT_CENTER_HIGH, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.CENTER, Level.HIGH),
    RIGHT_CENTER_MID(NTGridNode.RIGHT_CENTER_MID, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.CENTER, Level.MID),
    RIGHT_CENTER_LOW(NTGridNode.RIGHT_CENTER_LOW, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.CENTER, Level.LOW),

    RIGHT_RIGHT_HIGH(NTGridNode.RIGHT_RIGHT_HIGH, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.RIGHT, Level.HIGH),
    RIGHT_RIGHT_MID(NTGridNode.RIGHT_RIGHT_MID, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.RIGHT, Level.MID),
    RIGHT_RIGHT_LOW(NTGridNode.RIGHT_RIGHT_LOW, AlignmentZone.RIGHT, GenericDesiredAlignmentPosition.RIGHT, Level.LOW);

    public enum Level {
        LOW(SuperstructureStates.ElevatorState.ELEVATOR_STANDBY, SuperstructureStates.ClawState.CLAW_SHOOT_LOW),
        MID(SuperstructureStates.ElevatorState.ELEVATOR_EXTENDED_MID, SuperstructureStates.ClawState.CLAW_SHOOT_MID),
        HIGH(SuperstructureStates.ElevatorState.ELEVATOR_EXTENDED_HIGH, SuperstructureStates.ClawState.CLAW_SHOOT_HIGH);

        private final SuperstructureStates.ElevatorState elevatorState;
        private final SuperstructureStates.ClawState clawShootingState;

        Level(
                final SuperstructureStates.ElevatorState elevatorState,
                final SuperstructureStates.ClawState clawShootingState
        ) {
            this.elevatorState = elevatorState;
            this.clawShootingState = clawShootingState;
        }

        public SuperstructureStates.ElevatorState getElevatorState() {
            return elevatorState;
        }

        public SuperstructureStates.ClawState getClawShootingState() {
            return clawShootingState;
        }
    }

    private final NTGridNode ntGridNode;
    private final AlignmentZone alignmentZone;
    private final AlignmentZone.GenericDesiredAlignmentPosition alignmentPosition;
    private final Level level;

    public static final Map<NTGridNode, GridNode> ntToGridMap = Arrays.stream(values())
                    .collect(Collectors.toUnmodifiableMap(
                            GridNode::getNtGridNode,
                            gridNode -> gridNode
                    ));

    public static Optional<GridNode> getFromNT(final NTGridNode ntGridNode) {
        return Optional.ofNullable(ntToGridMap.get(ntGridNode));
    }

    GridNode(
            final NTGridNode ntGridNode,
            final AlignmentZone alignmentZone,
            final AlignmentZone.GenericDesiredAlignmentPosition alignmentPosition,
            final Level level
    ) {
        this.ntGridNode = ntGridNode;
        this.alignmentZone = alignmentZone;
        this.alignmentPosition = alignmentPosition;
        this.level = level;
    }

    public NTGridNode getNtGridNode() {
        return ntGridNode;
    }

    public AlignmentZone getAlignmentZone() {
        return AlignmentZone.AlignmentZoneMapping.getAlignmentZone(
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue),
                alignmentZone
        );
    }

    public AlignmentZone.GenericDesiredAlignmentPosition getAlignmentPosition() {
        return alignmentPosition;
    }

    public Level getLevel() {
        return level;
    }

    public static ElevatorClawCommand buildScoringSequence(
            final Elevator elevator,
            final Claw claw,
            final Level level
    ) {
        final SuperstructureStates.ElevatorState toLevelElevatorState = level.getElevatorState();
        return switch (level) {
            case HIGH, MID -> new ElevatorClawCommand.Builder(elevator, claw)
                    .withElevatorState(toLevelElevatorState)
//                    .waitUntilState(toLevelElevatorState)
                    .wait(0.25)
                    .withClawState(SuperstructureStates.ClawState.CLAW_DROP)
                    .waitUntilStates(toLevelElevatorState, SuperstructureStates.ClawState.CLAW_DROP)
                    .withClawState(SuperstructureStates.ClawState.CLAW_OUTTAKE)
                    .waitUntilState(SuperstructureStates.ClawState.CLAW_OUTTAKE)
                    .wait(0.6)
                    .withElevatorClawStates(
                            SuperstructureStates.ElevatorState.ELEVATOR_STANDBY,
                            SuperstructureStates.ClawState.CLAW_STANDBY
                    )
                    .build();
            case LOW -> new ElevatorClawCommand.Builder(elevator, claw)
                    .withElevatorClawStates(toLevelElevatorState, SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT)
                    .waitUntilState(SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT)
                    .withClawState(SuperstructureStates.ClawState.CLAW_SHOOT_LOW)
                    .waitUntilState(SuperstructureStates.ClawState.CLAW_SHOOT_LOW)
                    .wait(0.3)
                    .withClawState(SuperstructureStates.ClawState.CLAW_STANDBY)
                    .build();
        };
    }

    public static ElevatorClawCommand buildShootingSequence(
            final Elevator elevator,
            final Claw claw,
            final Level level
    ) {
        final SuperstructureStates.ClawState clawShootingState = level.getClawShootingState();
        return new ElevatorClawCommand.Builder(elevator, claw)
                .endIfNotInState(SuperstructureStates.ClawState.CLAW_ANGLE_SHOOT)
                .withClawState(clawShootingState)
                .waitUntilState(clawShootingState)
                .wait(0.3)
                .withClawState(SuperstructureStates.ClawState.CLAW_STANDBY)
                .build();
    }

    public ElevatorClawCommand buildScoringSequence(final Elevator elevator, final Claw claw) {
        return GridNode.buildScoringSequence(elevator, claw, level);
    }
}
