package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.SimConstants;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimPhoenix6CANCoder;
import frc.robot.utils.sim.motors.CTREPhoenix6TalonFXSim;

import java.util.List;

public class ElevatorSimSolver {
    private final ElevatorSim verticalElevatorSim;
    private final CANcoder verticalElevatorEncoder;
    private final CTREPhoenix6TalonFXSim verticalElevatorSimMotors;

    private double verticalStageOneExtension = 0;
    private double verticalStageTwoExtension = 0;

    private final ElevatorSim horizontalElevatorSim;
    private final CANcoder horizontalElevatorEncoder;
//    private final RevSparkMAXSim horizontalElevatorSimMotor;
    private final CTREPhoenix6TalonFXSim horizontalElevatorSimMotor;

    private double horizontalStageOneExtension = 0;
    private double horizontalStageTwoExtension = 0;

    public ElevatorSimSolver(
            final TalonFX verticalElevatorMotor,
            final TalonFX verticalElevatorMotorFollower,
            final CANcoder verticalElevatorEncoder,
//            final TitanSparkMAX horizontalElevatorMotor
            final TalonFX horizontalElevatorMotor,
            final CANcoder horizontalElevatorEncoder
    ) {
        // TODO: maybe a way to check if we're using FOC? or perhaps we can just assume we're always using FOC
        final DCMotor verticalElevatorDCMotors = SimUtils.getFalcon500FOC(2);
        this.verticalElevatorSim = new ElevatorSim(
                verticalElevatorDCMotors,
                SimConstants.Elevator.Vertical.GEARING,
                SimConstants.Elevator.Vertical.MOVING_MASS_KG,
                SimConstants.Elevator.Vertical.SPROCKET_RADIUS_M,
                SimConstants.Elevator.Vertical.MIN_TOTAL_EXT_M,
                SimConstants.Elevator.Vertical.MAX_TOTAL_EXT_M,
                SimConstants.Elevator.Vertical.SIMULATE_GRAVITY,
                0
        );

        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorSimMotors = new CTREPhoenix6TalonFXSim(
                List.of(verticalElevatorMotor, verticalElevatorMotorFollower),
                SimConstants.Elevator.Vertical.GEARING,
                new DCMotorSim(
                        verticalElevatorDCMotors,
                        SimConstants.Elevator.Vertical.GEARING,
                        SimConstants.Elevator.Vertical.EXT_MOI
                )
        );
        this.verticalElevatorSimMotors.attachFeedbackSensor(new SimPhoenix6CANCoder(verticalElevatorEncoder));

        final DCMotor horizontalElevatorDCMotor = DCMotor.getNEO(1);
        this.horizontalElevatorSim = new ElevatorSim(
                horizontalElevatorDCMotor,
                SimConstants.Elevator.Horizontal.GEARING,
                SimConstants.Elevator.Horizontal.MOVING_MASS_KG,
                SimConstants.Elevator.Horizontal.SPROCKET_RADIUS_M,
                SimConstants.Elevator.Horizontal.MIN_TOTAL_EXT_M,
                SimConstants.Elevator.Horizontal.MAX_TOTAL_EXT_M,
                SimConstants.Elevator.Horizontal.SIMULATE_GRAVITY,
                0
        );
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
//        this.horizontalElevatorSimMotor = new RevSparkMAXSim(
//                horizontalElevatorMotor,
//                horizontalElevatorDCMotor,
//                new DCMotorSim(
//                        horizontalElevatorDCMotor,
//                        SimConstants.Elevator.Horizontal.GEARING,
//                        SimConstants.Elevator.Horizontal.EXT_MOI
//                )
//        );
        this.horizontalElevatorSimMotor = new CTREPhoenix6TalonFXSim(
                horizontalElevatorMotor,
                SimConstants.Elevator.Horizontal.GEARING,
                new DCMotorSim(
                        horizontalElevatorDCMotor,
                        SimConstants.Elevator.Horizontal.GEARING,
                        SimConstants.Elevator.Horizontal.EXT_MOI
                )
        );
        this.horizontalElevatorSimMotor.attachFeedbackSensor(new SimPhoenix6CANCoder(horizontalElevatorEncoder));
    }

    private static double fromVerticalOutputRotationsToLinearDistanceMeters(final double outputRotations) {
        return outputRotations * SimConstants.Elevator.Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private static double fromVerticalLinearDistanceMetersToOutputRotations(final double linearDistance) {
        return linearDistance / SimConstants.Elevator.Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private static double fromHorizontalOutputRotationsToLinearDistanceMeters(final double outputRotations) {
        return outputRotations * SimConstants.Elevator.Horizontal.SPROCKET_CIRCUMFERENCE_M;
    }

    private static double fromHorizontalLinearDistanceMetersToOutputRotations(final double linearDistance) {
        return linearDistance / SimConstants.Elevator.Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private static void fillVerticalSimState(
            final ElevatorSimState elevatorSimState,
            final double verticalStageOneExtension,
            final double verticalStageTwoExtension
    ) {
        // Vertical Elevator stage one
        final Pose3d verticalStageOneLowerBoundPose = PoseUtils.withAxisOffset(
                elevatorSimState.elevatorRootPose,
                PoseUtils.Axis.Z,
                verticalStageOneExtension + Units.inchesToMeters(1.5)
        );

        final Pose3d verticalStageOneUpperBoundPose = PoseUtils.withAxisOffset(
                verticalStageOneLowerBoundPose,
                PoseUtils.Axis.Z,
                SimConstants.Elevator.Vertical.STAGE_ONE_HEIGHT
        );

        elevatorSimState.verticalStageOneLowerBoundPose = verticalStageOneLowerBoundPose;
        elevatorSimState.verticalStageOneUpperBoundPose = verticalStageOneUpperBoundPose;
        elevatorSimState.verticalStageOneCenterPose = new Pose3d(
                new Translation3d(
                        verticalStageOneLowerBoundPose.getX(),
                        verticalStageOneLowerBoundPose.getY(),
                        0.5 * (verticalStageOneLowerBoundPose.getZ() + verticalStageOneUpperBoundPose.getZ())
                ),
                verticalStageOneLowerBoundPose.getRotation()
        );

        // Vertical Elevator stage two
        final Pose3d verticalStageTwoLowerBoundPose = PoseUtils.withAxisOffset(
                verticalStageOneLowerBoundPose,
                PoseUtils.Axis.Z,
                verticalStageTwoExtension
        );

        final Pose3d verticalStageTwoUpperBoundPose = PoseUtils.withAxisOffset(
                verticalStageTwoLowerBoundPose,
                PoseUtils.Axis.Z,
                SimConstants.Elevator.Vertical.STAGE_TWO_HEIGHT
        );

        elevatorSimState.verticalStageTwoLowerBoundPose = verticalStageTwoLowerBoundPose;
        elevatorSimState.verticalStageTwoUpperBoundPose = verticalStageTwoUpperBoundPose;
        elevatorSimState.verticalStageTwoCenterPose = new Pose3d(
                new Translation3d(
                        verticalStageTwoLowerBoundPose.getX(),
                        verticalStageTwoLowerBoundPose.getY(),
                        0.5 * (verticalStageTwoLowerBoundPose.getZ() + verticalStageTwoUpperBoundPose.getZ())
                ),
                verticalStageTwoLowerBoundPose.getRotation()
        );
    }

    private static void fillHorizontalSimState(
            final ElevatorSimState elevatorSimState,
            final double horizontalStageOneExtension,
            final double horizontalStageTwoExtension
    ) {
        final Pose3d horizontalElevatorRoot = new Pose3d(
                elevatorSimState.verticalStageTwoCenterPose.getTranslation(),
                elevatorSimState.verticalStageTwoCenterPose.getRotation()
        );

        // Horizontal Stage One
        final Pose3d horizontalStageOneBackBoundPose = PoseUtils.withAxisOffset(
                horizontalElevatorRoot,
                PoseUtils.Axis.X,
                horizontalStageOneExtension
        );

        final Pose3d horizontalStageOneFrontBoundPose = PoseUtils.withAxisOffset(
                horizontalStageOneBackBoundPose,
                PoseUtils.Axis.X,
                SimConstants.Elevator.Horizontal.STAGE_ONE_LENGTH
        );

        elevatorSimState.horizontalRootPose = horizontalElevatorRoot;
        elevatorSimState.horizontalStageOneBackBoundPose = horizontalStageOneBackBoundPose;
        elevatorSimState.horizontalStageOneFrontBoundPose = horizontalStageOneFrontBoundPose;
        elevatorSimState.horizontalStageOneCenterPose = new Pose3d(
                new Translation3d(
                        0.5 * (horizontalStageOneBackBoundPose.getX() + horizontalStageOneFrontBoundPose.getX()),
                        horizontalStageOneBackBoundPose.getY(),
                        horizontalStageOneBackBoundPose.getZ()
                ),
                horizontalStageOneBackBoundPose.getRotation()
        );

        // Horizontal Stage Two
        final Pose3d horizontalStageTwoBackBoundPose = PoseUtils.withAxisOffset(
                horizontalStageOneBackBoundPose,
                PoseUtils.Axis.X,
                horizontalStageTwoExtension
        );

        final Pose3d horizontalStageTwoFrontBoundPose = PoseUtils.withAxisOffset(
                horizontalStageTwoBackBoundPose,
                PoseUtils.Axis.X,
                SimConstants.Elevator.Horizontal.STAGE_TWO_LENGTH
        );

        elevatorSimState.horizontalStageTwoBackBoundPose = horizontalStageTwoBackBoundPose;
        elevatorSimState.horizontalStageTwoFrontBoundPose = horizontalStageTwoFrontBoundPose;
        elevatorSimState.horizontalStageTwoCenterPose = new Pose3d(
                new Translation3d(
                        0.5 * (horizontalStageTwoBackBoundPose.getX() + horizontalStageTwoFrontBoundPose.getX()),
                        horizontalStageTwoBackBoundPose.getY(),
                        horizontalStageTwoBackBoundPose.getZ()
                ),
                horizontalStageTwoBackBoundPose.getRotation()
        );
    }

    private static ElevatorSimState generateSimState(
            final double verticalStageOneExtension,
            final double verticalStageTwoExtension,
            final double horizontalStageOneExtension,
            final double horizontalStageTwoExtension
    ) {
        final ElevatorSimState elevatorSimState = new ElevatorSimState();
        elevatorSimState.elevatorRootPose = SimConstants.Elevator.Vertical.ROBOT_TO_ROOT_MOUNT_POSE;

        fillVerticalSimState(elevatorSimState, verticalStageOneExtension, verticalStageTwoExtension);
        fillHorizontalSimState(elevatorSimState, horizontalStageOneExtension, horizontalStageTwoExtension);

        return elevatorSimState;
    }

    private static double cascadeFromPreviousStage(
            final double previousStageLinearDistance,
            final double previousStageOffset,
            final double previousStageMaxExtension,
            final double nextStageOffset,
            final double nextStageMaxExtension
    ) {
        return (((previousStageLinearDistance + previousStageOffset)
                * (nextStageMaxExtension + nextStageOffset)
        ) / (previousStageMaxExtension + previousStageOffset)) - nextStageOffset;
    }

    private static double computeVerticalStageOneExtension(final double verticalOutputPositionRots) {
        return fromVerticalOutputRotationsToLinearDistanceMeters(
                verticalOutputPositionRots
        );
    }

    private static double computeVerticalStageTwoExtension(final double verticalStageOneExtension) {
        return cascadeFromPreviousStage(
                verticalStageOneExtension,
                SimConstants.Elevator.Vertical.STAGE_ONE_OFFSET,
                SimConstants.Elevator.Vertical.STAGE_ONE_EXT_HEIGHT,
                SimConstants.Elevator.Vertical.STAGE_TWO_OFFSET,
                SimConstants.Elevator.Vertical.STAGE_TWO_EXT_HEIGHT
        );
    }

    private static double computeHorizontalStageOneExtension(final double horizontalOutputPositionRots) {
        return fromHorizontalOutputRotationsToLinearDistanceMeters(
                horizontalOutputPositionRots
        );
    }

    private static double computeHorizontalStageTwoExtension(final double horizontalStageOneExtension) {
        return cascadeFromPreviousStage(
                horizontalStageOneExtension,
                SimConstants.Elevator.Horizontal.STAGE_ONE_OFFSET,
                SimConstants.Elevator.Horizontal.STAGE_ONE_EXT_LENGTH,
                SimConstants.Elevator.Horizontal.STAGE_TWO_OFFSET,
                SimConstants.Elevator.Horizontal.STAGE_TWO_EXT_LENGTH
        );
    }

    public static Elevator.ElevatorPoseState generatePoseState(
            final double verticalOutputPositionRots,
            final double horizontalOutputPositionRots
    ) {
        final double verticalStageOneExtension = computeVerticalStageOneExtension(verticalOutputPositionRots);
        final double horizontalStageOneExtension = computeHorizontalStageOneExtension(horizontalOutputPositionRots);
        return generateSimState(
                verticalStageOneExtension,
                computeVerticalStageTwoExtension(verticalStageOneExtension),
                horizontalStageOneExtension,
                computeHorizontalStageTwoExtension(horizontalStageOneExtension)
        ).toElevatorPoseState();
    }

    private void updateExtensions(
            final double verticalOutputPositionRots,
            final double horizontalOutputPositionRots
    ) {
        this.verticalStageOneExtension = computeVerticalStageOneExtension(verticalOutputPositionRots);
        this.verticalStageTwoExtension = computeVerticalStageTwoExtension(verticalStageOneExtension);

        this.horizontalStageOneExtension = computeHorizontalStageOneExtension(horizontalOutputPositionRots);
        this.horizontalStageTwoExtension = computeHorizontalStageTwoExtension(horizontalStageOneExtension);
    }

    public double getVerticalElevatorPosition() {
        return Phoenix6Utils.latencyCompensateRefreshedSignalIfIsGood(
                verticalElevatorEncoder.getPosition().refresh(),
                verticalElevatorEncoder.getVelocity().refresh()
        );
    }

    public double getHorizontalElevatorPosition() {
        return Phoenix6Utils.latencyCompensateRefreshedSignalIfIsGood(
                horizontalElevatorEncoder.getPosition().refresh(),
                horizontalElevatorEncoder.getVelocity().refresh()
        );
    }

    public void update(final double dt) {
        verticalElevatorSimMotors.rawUpdate(
                fromVerticalLinearDistanceMetersToOutputRotations(verticalElevatorSim.getPositionMeters()),
                fromVerticalLinearDistanceMetersToOutputRotations(verticalElevatorSim.getVelocityMetersPerSecond())
        );
        verticalElevatorSim.setInputVoltage(verticalElevatorSimMotors.getMotorVoltage());
        verticalElevatorSim.update(dt);

        horizontalElevatorSimMotor.rawUpdate(
                fromHorizontalLinearDistanceMetersToOutputRotations(horizontalElevatorSim.getPositionMeters()),
                fromHorizontalLinearDistanceMetersToOutputRotations(horizontalElevatorSim.getVelocityMetersPerSecond())
        );
        horizontalElevatorSim.setInputVoltage(horizontalElevatorSimMotor.getMotorVoltage());
        horizontalElevatorSim.update(dt);

        updateExtensions(getVerticalElevatorPosition(), getHorizontalElevatorPosition());
    }

    public Elevator.ElevatorPoseState getElevatorPoseState() {
        return generateSimState(
                verticalStageOneExtension,
                verticalStageTwoExtension,
                horizontalStageOneExtension,
                horizontalStageTwoExtension
        ).toElevatorPoseState();
    }

    @SuppressWarnings("unused")
    private static class ElevatorSimState {
        private Pose3d elevatorRootPose;
        private Pose3d verticalStageOneLowerBoundPose;
        private Pose3d verticalStageOneCenterPose;
        private Pose3d verticalStageOneUpperBoundPose;
        private Pose3d verticalStageTwoLowerBoundPose;
        private Pose3d verticalStageTwoCenterPose;
        private Pose3d verticalStageTwoUpperBoundPose;
        private Pose3d horizontalRootPose;
        private Pose3d horizontalStageOneCenterPose;
        private Pose3d horizontalStageOneBackBoundPose;
        private Pose3d horizontalStageOneFrontBoundPose;
        private Pose3d horizontalStageTwoCenterPose;
        private Pose3d horizontalStageTwoBackBoundPose;
        private Pose3d horizontalStageTwoFrontBoundPose;

        private Elevator.ElevatorPoseState toElevatorPoseState() {
            return new Elevator.ElevatorPoseState(
                    elevatorRootPose,
                    verticalStageOneCenterPose,
                    verticalStageTwoCenterPose,
                    horizontalRootPose,
                    horizontalStageOneCenterPose,
                    horizontalStageTwoCenterPose,
                    horizontalStageTwoFrontBoundPose
            );
        }
    }
}
