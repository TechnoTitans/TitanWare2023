package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import static frc.robot.Constants.Sim.*;

public class ElevatorSimSolver {
    private final double ELEVATOR_VERTICAL_EXTENSION_RATIO =
            ELEVATOR_VERTICAL_STAGE_ONE_EXT_HEIGHT / ELEVATOR_VERTICAL_STAGE_TWO_EXT_HEIGHT;

    private final Pose3d elevatorRootPose;
    private Pose3d elevatorStageOneCenterPose;
    private Pose3d elevatorStageOneLowerBoundPose;
    private Pose3d elevatorStageOneUpperBoundPose;

    private Pose3d elevatorStageTwoCenterPose;
    private Pose3d elevatorStageTwoLowerBoundPose;
    private Pose3d elevatorStageTwoUpperBoundPose;


    public ElevatorSimSolver() {
        this.elevatorRootPose = new Pose3d(0, 0, 0, new Rotation3d());
        this.elevatorStageOneLowerBoundPose = new Pose3d(
                elevatorRootPose.getTranslation(), elevatorRootPose.getRotation()
        );

        this.elevatorStageOneUpperBoundPose = new Pose3d(
                elevatorStageOneLowerBoundPose.getX(),
                elevatorStageOneLowerBoundPose.getY(),
                elevatorStageOneLowerBoundPose.getZ() + ELEVATOR_VERTICAL_STAGE_ONE_HEIGHT,
                elevatorStageOneLowerBoundPose.getRotation()
        );

        this.elevatorStageOneCenterPose = elevatorStageOneLowerBoundPose.interpolate(
                elevatorStageOneUpperBoundPose, 0.5
        );

        this.elevatorStageTwoLowerBoundPose = new Pose3d(
                elevatorStageOneLowerBoundPose.getTranslation(), elevatorStageOneLowerBoundPose.getRotation()
        );

        this.elevatorStageTwoUpperBoundPose = new Pose3d(
                elevatorStageTwoLowerBoundPose.getX(),
                elevatorStageTwoLowerBoundPose.getY(),
                elevatorStageTwoLowerBoundPose.getZ() + ELEVATOR_VERTICAL_STAGE_TWO_HEIGHT,
                elevatorStageTwoLowerBoundPose.getRotation()
        );

        this.elevatorStageTwoCenterPose = elevatorStageTwoLowerBoundPose.interpolate(
                elevatorStageTwoUpperBoundPose, 0.5
        );
    }

    public void update(final double desiredVerticalHeight, final double horizontalExtensionHeight) {
        final double stageTwoExtension =
                (desiredVerticalHeight - (0.5 * ELEVATOR_VERTICAL_STAGE_TWO_HEIGHT))
                        / (1 + ELEVATOR_VERTICAL_EXTENSION_RATIO);

        final double stageOneExtension = stageTwoExtension * ELEVATOR_VERTICAL_EXTENSION_RATIO;
    }
}
