package frc.robot.utils.alignment;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

public enum NTGridNode {
    LEFT_LEFT_HIGH(0),
    LEFT_LEFT_MID(1),
    LEFT_LEFT_LOW(2),

    LEFT_CENTER_HIGH(3),
    LEFT_CENTER_MID(4),
    LEFT_CENTER_LOW(5),

    LEFT_RIGHT_HIGH(6),
    LEFT_RIGHT_MID(7),
    LEFT_RIGHT_LOW(8),

    //No 9, only 9 nodes per row on grid

    CENTER_LEFT_HIGH(10),
    CENTER_LEFT_MID(11),
    CENTER_LEFT_LOW(12),

    CENTER_CENTER_HIGH(13),
    CENTER_CENTER_MID(14),
    CENTER_CENTER_LOW(15),

    CENTER_RIGHT_HIGH(16),
    CENTER_RIGHT_MID(17),
    CENTER_RIGHT_LOW(18),

    //No 19

    RIGHT_LEFT_HIGH(20),
    RIGHT_LEFT_MID(21),
    RIGHT_LEFT_LOW(22),

    RIGHT_CENTER_HIGH(23),
    RIGHT_CENTER_MID(24),
    RIGHT_CENTER_LOW(25),

    RIGHT_RIGHT_HIGH(26),
    RIGHT_RIGHT_MID(27),
    RIGHT_RIGHT_LOW(28),

    UNKNOWN(-1);

    private static final Map<Long, NTGridNode> idGridNodeMap = Arrays.stream(values())
                    .collect(Collectors.toUnmodifiableMap(
                            NTGridNode::getNtID,
                            gridNode -> gridNode
                    ));

    private final long ntID;

    NTGridNode(final long ntID) {
        this.ntID = ntID;
    }

    public long getNtID() {
        return ntID;
    }

    public static NTGridNode fromNtID(final long ntID) {
        return idGridNodeMap.get(ntID);
    }
}
