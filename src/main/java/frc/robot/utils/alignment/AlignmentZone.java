package frc.robot.utils.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.PoseUtils;

import java.util.*;
import java.util.stream.Collectors;

public enum AlignmentZone {
    LEFT(
            new Translation2d(1, 0),
            new Translation2d(3.35, 1.89),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 1.63), Rotation2d.fromDegrees(180)),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 1.05), Rotation2d.fromDegrees(180)),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 0.47), Rotation2d.fromDegrees(180))
    ),
    CENTER(
            new Translation2d(1, 1.91),
            new Translation2d(3.35, 3.58),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 3.38), Rotation2d.fromDegrees(180)),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 2.72), Rotation2d.fromDegrees(180)),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 2.2), Rotation2d.fromDegrees(180))
    ),
    RIGHT(
            new Translation2d(1, 3.6),
            new Translation2d(3.35, 5.4585),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 5.045), Rotation2d.fromDegrees(180)),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 4.41), Rotation2d.fromDegrees(180)),
            new Pose2d(new Translation2d(FieldConstants.GRID_SCORING_X_POSITION, 3.84), Rotation2d.fromDegrees(180))
    ),
    SUBSTATION(
            new Translation2d(14.5, 5.52),
            new Translation2d(16.5, 8),
            new Pose2d(new Translation2d(FieldConstants.SUBSTATION_PICKUP_X_POSITION, 7.36), Rotation2d.fromDegrees(0)),
            new Pose2d(new Translation2d(FieldConstants.SUBSTATION_PICKUP_X_POSITION, 6), Rotation2d.fromDegrees(0))
    ),
    CHARGE_STATION_RIGHT_TRAJECTORY_ZONE(
            new Translation2d(4.89, 4),
            new Translation2d(9.85, 8),
            AlignmentZoneType.TRAJECTORY_GRID,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(3.9, 4.8), Rotation2d.fromDegrees(180)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(5.83, 0.75), Rotation2d.fromDegrees(180))
            )
    ),
    CHARGE_STATION_RIGHT_FAR_TRAJECTORY_ZONE(
            new Translation2d(9.85, 4),
            new Translation2d(13.18, 6.8),
            AlignmentZoneType.TRAJECTORY_GRID,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(3.9, 4.8), Rotation2d.fromDegrees(180)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(5.83, 0.75), Rotation2d.fromDegrees(180))
            )
    ),
    CHARGE_STATION_RIGHT_NEAR_TRAJECTORY_ZONE(
            new Translation2d(3.3, 4),
            new Translation2d(4.89, 8),
            AlignmentZoneType.TRAJECTORY_GRID,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(3.9, 4.8), Rotation2d.fromDegrees(180)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(3.9, 4.8), Rotation2d.fromDegrees(180))
            )
    ),
    CHARGE_STATION_CENTER_TRAJECTORY_ZONE(
            new Translation2d(4.9, 1.49),
            new Translation2d(11.64, 4),
            AlignmentZoneType.TRAJECTORY_GRID,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(5.83, 4.8), Rotation2d.fromDegrees(180)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(5.83, 0.75), Rotation2d.fromDegrees(180))
            )
    ),
    CHARGE_STATION_LEFT_TRAJECTORY_ZONE(
            new Translation2d(4.9, 0),
            new Translation2d(11.64, 1.49),
            AlignmentZoneType.TRAJECTORY_GRID,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(5.83, 4.8), Rotation2d.fromDegrees(180)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(3.9, 0.75), Rotation2d.fromDegrees(180))
            )
    ),
    CHARGE_STATION_LEFT_NEAR_TRAJECTORY_ZONE(
            new Translation2d(2.89, 0),
            new Translation2d(4.89, 1.47),
            AlignmentZoneType.TRAJECTORY_GRID,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(2.75, 0.75), Rotation2d.fromDegrees(180)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(2.75, 0.75), Rotation2d.fromDegrees(180))
            )
    ),
    SUBSTATION_TRAJECTORY_ZONE(
            new Translation2d(13.17, 5.52),
            new Translation2d(16.5, 8),
            AlignmentZoneType.TRAJECTORY_SUBSTATION,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(14.25, 7.36), Rotation2d.fromDegrees(0)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(14.25, 6), Rotation2d.fromDegrees(0))
            )
    ),
    SUBSTATION_ALT_TRAJECTORY_ZONE(
            new Translation2d(9.85, 6.8),
            new Translation2d(13.18, 8),
            AlignmentZoneType.TRAJECTORY_SUBSTATION,
            Map.of(
                    TrajectoryAlignmentSide.LEFT, new Pose2d(new Translation2d(14.25, 7.36), Rotation2d.fromDegrees(0)),
                    TrajectoryAlignmentSide.RIGHT, new Pose2d(new Translation2d(14.25, 6), Rotation2d.fromDegrees(0))
            )
    );

    public enum AlignmentZoneKind {
        DIRECT,
        TRAJECTORY
    }

    public enum AlignmentZoneType {
        GRID(AlignmentZoneKind.DIRECT),
        SUBSTATION(AlignmentZoneKind.DIRECT),
        TRAJECTORY_GRID(AlignmentZoneKind.TRAJECTORY),
        TRAJECTORY_SUBSTATION(AlignmentZoneKind.TRAJECTORY);

        private final AlignmentZoneKind alignmentZoneKind;

        AlignmentZoneType(final AlignmentZoneKind alignmentZoneKind) {
            this.alignmentZoneKind = alignmentZoneKind;
        }

        public AlignmentZoneKind getAlignmentZoneKind() {
            return alignmentZoneKind;
        }
    }

    public enum NodePosition {
        LEFT_CONE(GenericDesiredAlignmentPosition.LEFT, AlignmentZoneType.GRID),
        CENTER_CUBE(GenericDesiredAlignmentPosition.CENTER, AlignmentZoneType.GRID),
        RIGHT_CONE(GenericDesiredAlignmentPosition.RIGHT, AlignmentZoneType.GRID),
        LEFT_DOUBLE_SUBSTATION(GenericDesiredAlignmentPosition.LEFT, AlignmentZoneType.SUBSTATION),
        RIGHT_DOUBLE_SUBSTATION(GenericDesiredAlignmentPosition.RIGHT, AlignmentZoneType.SUBSTATION);

        private final GenericDesiredAlignmentPosition genericDesiredAlignmentPosition;
        private final AlignmentZoneType alignmentZoneType;

        private static final Map<AlignmentZoneType, Map<GenericDesiredAlignmentPosition, NodePosition>> map =
                Arrays.stream(NodePosition.values()).collect(
                        Collectors.toUnmodifiableMap(
                                NodePosition::getAlignmentZoneType,
                                nodePosition -> new HashMap<>(
                                        Map.of(nodePosition.getGenericDesiredAlignmentPosition(), nodePosition)
                                ),
                                (existingMap, nextMap) -> {
                                    existingMap.putAll(nextMap);
                                    return existingMap;
                                }
                        )
                );

        NodePosition(
                final GenericDesiredAlignmentPosition genericDesiredAlignmentPosition,
                final AlignmentZoneType alignmentZoneType
        ) {
            this.genericDesiredAlignmentPosition = genericDesiredAlignmentPosition;
            this.alignmentZoneType = alignmentZoneType;
        }

        public GenericDesiredAlignmentPosition getGenericDesiredAlignmentPosition() {
            return genericDesiredAlignmentPosition;
        }

        public AlignmentZoneType getAlignmentZoneType() {
            return alignmentZoneType;
        }

        public static NodePosition getFromAlignmentPositionAndZoneType(
                final GenericDesiredAlignmentPosition alignmentPosition,
                final AlignmentZoneType zoneType
        ) {
            return map.get(zoneType).get(alignmentPosition);
        }
    }

    public enum GenericDesiredAlignmentPosition {
        LEFT,
        CENTER,
        RIGHT,
        SINGLE_SUB
    }

    public enum AlignmentZoneMapping {
        LEFT(AlignmentZone.LEFT, AlignmentZone.RIGHT),
        CENTER(AlignmentZone.CENTER, AlignmentZone.CENTER),
        RIGHT(AlignmentZone.RIGHT, AlignmentZone.LEFT);

        private final HashSet<DriverStation.Alliance> mapOnAlliances;
        private final AlignmentZone mapFromAlignmentZone;
        private final AlignmentZone mapToAlignmentZone;

        private static final Map<AlignmentZone, AlignmentZoneMapping> map =
                Arrays.stream(AlignmentZoneMapping.values()).collect(
                        Collectors.toUnmodifiableMap(AlignmentZoneMapping::getMapFromAlignmentZone, zone -> zone)
                );

        public static AlignmentZone getAlignmentZone(
                final DriverStation.Alliance currentAlliance,
                final AlignmentZone alignmentZone
        ) {
            final AlignmentZoneMapping mapping = map.get(alignmentZone);
            return (mapping != null && mapping.shouldMapOnAlliance(currentAlliance))
                    ? mapping.getMapToAlignmentZone()
                    : alignmentZone;
        }

        AlignmentZoneMapping(
                final HashSet<DriverStation.Alliance> mapOnAlliances,
                final AlignmentZone mapFromAlignmentZone,
                final AlignmentZone mapToAlignmentZone
        ) {
            this.mapOnAlliances = mapOnAlliances;
            this.mapFromAlignmentZone = mapFromAlignmentZone;
            this.mapToAlignmentZone = mapToAlignmentZone;
        }

        AlignmentZoneMapping(
                final DriverStation.Alliance mapOnAlliance,
                final AlignmentZone mapFromAlignmentZone,
                final AlignmentZone mapToAlignmentZone
        ) {
            this(new HashSet<>(Set.of(mapOnAlliance)), mapFromAlignmentZone, mapToAlignmentZone);
        }

        AlignmentZoneMapping(
                final AlignmentZone mapFromAlignmentZone,
                final AlignmentZone mapToAlignmentZone
        ) {
            this(DriverStation.Alliance.Red, mapFromAlignmentZone, mapToAlignmentZone);
        }

        public AlignmentZone getMapToAlignmentZone() {
            return mapToAlignmentZone;
        }
        public AlignmentZone getMapFromAlignmentZone() {
            return mapFromAlignmentZone;
        }
        public boolean shouldMapOnAlliance(final DriverStation.Alliance alliance) {
            return mapOnAlliances.contains(alliance);
        }
    }

    public enum TrajectoryAlignmentSide {
        LEFT,
        RIGHT,
        SINGLE_SUBSTATION
    }

    public enum TrajectoryAlignmentSideMapping {
        LEFT(TrajectoryAlignmentSide.LEFT, TrajectoryAlignmentSide.RIGHT),
        RIGHT(TrajectoryAlignmentSide.RIGHT, TrajectoryAlignmentSide.LEFT);

        private final HashSet<DriverStation.Alliance> mapOnAlliances;

        private final TrajectoryAlignmentSide mapFromAlignmentSide;
        private final TrajectoryAlignmentSide mapToAlignmentSide;

        private static final Map<TrajectoryAlignmentSide, TrajectoryAlignmentSideMapping> map =
                Arrays.stream(TrajectoryAlignmentSideMapping.values()).collect(
                        Collectors.toUnmodifiableMap(
                                TrajectoryAlignmentSideMapping::getMapFromAlignmentSide,
                                zone -> zone
                        )
                );

        public static TrajectoryAlignmentSide getTrajectoryAlignmentSide(
                final DriverStation.Alliance currentAlliance,
                final TrajectoryAlignmentSide trajectoryAlignmentSide
        ) {
            final TrajectoryAlignmentSideMapping mapping = map.get(trajectoryAlignmentSide);
            return (mapping != null && mapping.shouldMapOnAlliance(currentAlliance))
                    ? mapping.getMapToAlignmentSide()
                    : trajectoryAlignmentSide;
        }

        TrajectoryAlignmentSideMapping(
                final HashSet<DriverStation.Alliance> mapOnAlliances,
                final TrajectoryAlignmentSide mapFromAlignmentSide,
                final TrajectoryAlignmentSide mapToAlignmentSide
        ) {
            this.mapOnAlliances = mapOnAlliances;
            this.mapFromAlignmentSide = mapFromAlignmentSide;
            this.mapToAlignmentSide = mapToAlignmentSide;
        }

        TrajectoryAlignmentSideMapping(
                final DriverStation.Alliance mapOnAlliance,
                final TrajectoryAlignmentSide mapFromAlignmentSide,
                final TrajectoryAlignmentSide mapToAlignmentSide
        ) {
            this(new HashSet<>(Set.of(mapOnAlliance)), mapFromAlignmentSide, mapToAlignmentSide);
        }

        TrajectoryAlignmentSideMapping(
                final TrajectoryAlignmentSide mapFromAlignmentSide,
                final TrajectoryAlignmentSide mapToAlignmentSide
        ) {
            this(DriverStation.Alliance.Red, mapFromAlignmentSide, mapToAlignmentSide);
        }

        public TrajectoryAlignmentSide getMapFromAlignmentSide() {
            return mapFromAlignmentSide;
        }

        public TrajectoryAlignmentSide getMapToAlignmentSide() {
            return mapToAlignmentSide;
        }

        public boolean shouldMapOnAlliance(final DriverStation.Alliance alliance) {
            return mapOnAlliances.contains(alliance);
        }
    }

    private static final double CENTER_MIN_Y = Math.min(CENTER.cornerBLBounds.getY(), CENTER.cornerTRBounds.getY());
    public static final double GRID_CENTER_Y =
            CENTER_MIN_Y + ((Math.max(CENTER.cornerBLBounds.getY(), CENTER.cornerTRBounds.getY()) - CENTER_MIN_Y) / 2);
    public static final double GRID_CENTER_Y_BLUE = FieldConstants.LOADING_ZONE_WIDTH_Y_METERS + GRID_CENTER_Y;
    public static final double GRID_CENTER_Y_RED = GRID_CENTER_Y_BLUE + GRID_CENTER_Y;

    public static final AlignmentZone[] cachedValues = values();

    private final Translation2d cornerBLBounds;
    private final Translation2d cornerTRBounds;

    private final AlignmentZoneType alignmentZoneType;
    private final PoseUtils.MirroringBehavior mirroringBehavior;

    private final Pose2d leftCone;
    private final Pose2d centerCube;
    private final Pose2d rightCone;

    private final Pose2d leftDoubleSubstation;
    private final Pose2d rightDoubleSubstation;

    private final Map<TrajectoryAlignmentSide, Pose2d> trajectoryTargetMap;

    private Pose2d[] loggablePoseRegionArray;
    private DriverStation.Alliance lastLoggedAlliance;

    AlignmentZone(
            final Translation2d cornerBLBounds,
            final Translation2d cornerTRBounds,
            final AlignmentZoneType alignmentZoneType,
            final PoseUtils.MirroringBehavior mirroringBehavior,
            final Pose2d leftCone,
            final Pose2d centerCube,
            final Pose2d rightCone,
            final Pose2d leftDoubleSubstation,
            final Pose2d rightDoubleSubstation,
            final Map<TrajectoryAlignmentSide, Pose2d> trajectoryTargetMap
    ) {
        this.cornerBLBounds = cornerBLBounds;
        this.cornerTRBounds = cornerTRBounds;

        this.alignmentZoneType = alignmentZoneType;
        this.mirroringBehavior = mirroringBehavior;

        this.leftCone = leftCone;
        this.centerCube = centerCube;
        this.rightCone = rightCone;

        this.leftDoubleSubstation = leftDoubleSubstation;
        this.rightDoubleSubstation = rightDoubleSubstation;
        this.trajectoryTargetMap = trajectoryTargetMap;
    }

    AlignmentZone(
            final Translation2d cornerBLBounds,
            final Translation2d cornerTRBounds,
            final Pose2d leftCone,
            final Pose2d centerCube,
            final Pose2d rightCone
    ) {
        this(
                cornerBLBounds,
                cornerTRBounds,
                AlignmentZoneType.GRID,
                PoseUtils.MirroringBehavior.MIRROR_ACROSS_GRID_CENTER_POINT,
                leftCone,
                centerCube,
                rightCone,
                new Pose2d(),
                new Pose2d(),
                Map.of()
        );
    }

    AlignmentZone(
            final Translation2d cornerBLBounds,
            final Translation2d cornerTRBounds,
            final Pose2d leftDoubleSubstation,
            final Pose2d rightDoubleSubstation
    ) {
        this(
                cornerBLBounds,
                cornerTRBounds,
                AlignmentZoneType.SUBSTATION,
                PoseUtils.MirroringBehavior.MIRROR_ACROSS_X_CENTER,
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                leftDoubleSubstation,
                rightDoubleSubstation,
                Map.of()
        );
    }

    AlignmentZone(
            final Translation2d cornerBLBounds,
            final Translation2d cornerTRBounds,
            final AlignmentZoneType alignmentZoneType,
            final Map<TrajectoryAlignmentSide, Pose2d> trajectoryTargetMap
    ) {
        this(
                cornerBLBounds,
                cornerTRBounds,
                alignmentZoneType,
                PoseUtils.MirroringBehavior.MIRROR_ACROSS_X_CENTER,
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                trajectoryTargetMap
        );
    }

    public Translation2d getCornerBLBounds() {
        return cornerBLBounds;
    }

    public Translation2d getCornerTRBounds() {
        return cornerTRBounds;
    }

    public Pose2d getLeftCone() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return leftCone;
        } else {
            return PoseUtils.reflectAndLocalizeGridPoseToAlliance(
                    rightCone, DriverStation.Alliance.Blue, DriverStation.Alliance.Red
            );
        }
    }

    public Pose2d getCenterCube() {
        return centerCube;
    }

    public Pose2d getRightCone() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return rightCone;
        } else {
            return PoseUtils.reflectAndLocalizeGridPoseToAlliance(
                    leftCone, DriverStation.Alliance.Blue, DriverStation.Alliance.Red
            );
        }
    }

    public Pose2d getLeftDoubleSubstation() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return leftDoubleSubstation;
        } else {
            return PoseUtils.localizePoseOnAlliance(
                    rightDoubleSubstation, DriverStation.Alliance.Blue, getMirroringBehavior()
            );
        }
    }

    public Pose2d getRightDoubleSubstation() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return rightDoubleSubstation;
        } else {
            return PoseUtils.localizePoseOnAlliance(
                    leftDoubleSubstation, DriverStation.Alliance.Blue, getMirroringBehavior()
            );
        }
    }

    public Pose2d getTrajectoryTarget(final TrajectoryAlignmentSide trajectoryAlignmentSide) {
        final DriverStation.Alliance alliance = DriverStation.getAlliance();
        final TrajectoryAlignmentSide mappedTrajectoryAlignmentSide = TrajectoryAlignmentSideMapping
                .getTrajectoryAlignmentSide(alliance, trajectoryAlignmentSide);

        final Pose2d trajectoryTargetPose = trajectoryTargetMap.get(mappedTrajectoryAlignmentSide);
        if (alliance == DriverStation.Alliance.Blue) {
            return trajectoryTargetPose;
        } else {
            return PoseUtils.localizePoseOnAlliance(
                    trajectoryTargetPose, DriverStation.Alliance.Blue, getMirroringBehavior()
            );
        }
    }

    public AlignmentZoneType getAlignmentZoneType() {
        return alignmentZoneType;
    }

    public PoseUtils.MirroringBehavior getMirroringBehavior() {
        return mirroringBehavior;
    }

    public Pose2d getAlignmentPosition(final GenericDesiredAlignmentPosition genericDesiredAlignmentPosition) {
        final NodePosition nodePosition = NodePosition.getFromAlignmentPositionAndZoneType(
                genericDesiredAlignmentPosition, getAlignmentZoneType()
        );

        return switch (nodePosition) {
            case LEFT_CONE -> getLeftCone();
            case CENTER_CUBE -> getCenterCube();
            case RIGHT_CONE -> getRightCone();
            case LEFT_DOUBLE_SUBSTATION -> getLeftDoubleSubstation();
            case RIGHT_DOUBLE_SUBSTATION -> getRightDoubleSubstation();
        };
    }

    public static Pose2d[] makeLoggablePoseRegionArray(
            final Translation2d cornerBLBounds,
            final Translation2d cornerTRBounds
    ) {
        final Translation2d yDiff =  new Translation2d(0, cornerBLBounds.minus(cornerTRBounds).getY());
        final Translation2d[] translationArray = new Translation2d[] {
                cornerBLBounds,
                cornerBLBounds.minus(yDiff),
                cornerTRBounds,
                cornerTRBounds.plus(yDiff),
                cornerBLBounds
        };

        return Arrays.stream(translationArray)
                .map(translation2d -> new Pose2d(translation2d, Rotation2d.fromDegrees(0)))
                .toArray(Pose2d[]::new);
    }

    public Pose2d[] getLoggablePoseRegionArray() {
        final DriverStation.Alliance alliance = DriverStation.getAlliance();
        return Objects.requireNonNullElseGet(
                alliance != lastLoggedAlliance ? null : loggablePoseRegionArray,
                () -> {
                    lastLoggedAlliance = alliance;
                    loggablePoseRegionArray = makeLoggablePoseRegionArray(
                            PoseUtils.localizeTranslationOnAlliance(
                                    getCornerBLBounds(), DriverStation.Alliance.Blue, getMirroringBehavior()
                            ),
                            PoseUtils.localizeTranslationOnAlliance(
                                    getCornerTRBounds(), DriverStation.Alliance.Blue, getMirroringBehavior()
                            )
                    );

                    return loggablePoseRegionArray;
                }
        );
    }

    public static AlignmentZone getAlignmentZoneFromCurrentPose(final Pose2d currentPose, final boolean ignoreMapping) {
        final DriverStation.Alliance alliance = DriverStation.getAlliance();
        for (final AlignmentZone zone : cachedValues) {
            if (PoseUtils.poseWithinArea(
                    currentPose, zone.getCornerBLBounds(), zone.getCornerTRBounds(), zone.getMirroringBehavior()
            )) {
                return ignoreMapping ? zone : AlignmentZoneMapping.getAlignmentZone(alliance, zone);
            }
        }

        return null;
    }

    public static AlignmentZone getAlignmentZoneFromCurrentPose(final Pose2d currentPose) {
        return getAlignmentZoneFromCurrentPose(currentPose, false);
    }

    /**
     * Check if a provided {@link Pose2d} is inside a specified {@link AlignmentZone}
     * <p>
     * Note that this function is potentially somewhat expensive to call within a tight loop depending on its impl
     * @param currentPose the current {@link Pose2d} to check against
     * @param alignmentZone the {@link AlignmentZone} to check the pose against
     * @return true if the {@link Pose2d} is inside the {@link AlignmentZone}, false if not
     */
    public static boolean isPoseInAlignmentZone(final Pose2d currentPose, final AlignmentZone alignmentZone) {
        return PoseUtils.poseWithinArea(
                currentPose,
                alignmentZone.getCornerBLBounds(),
                alignmentZone.getCornerTRBounds(),
                alignmentZone.getMirroringBehavior()
        );
    }
}
