package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.feedback.SimPhoenix6CANCoder;
import frc.robot.utils.sim.motors.CTREPhoenix6TalonFXSim;
import frc.robot.utils.sim.motors.RevSparkMAXSim;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class ElevatorSimSolver {
    private final Pose3d elevatorRootPose;

    // Vertical Elevator Motor Sims
    private final ElevatorSim verticalElevatorSim;
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final CANcoder verticalElevatorEncoder;
    private final CTREPhoenix6TalonFXSim verticalElevatorSimMotors;

    // Vertical Elevator Stage One
    private double verticalStageOneExtension = 0;
    private Pose3d verticalStageOneLowerBoundPose = new Pose3d();
    private Pose3d verticalStageOneCenterPose = new Pose3d();
    private Pose3d verticalStageOneUpperBoundPose = new Pose3d();

    // Vertical Elevator Stage Two
    private double verticalStageTwoExtension = 0;
    private Pose3d verticalStageTwoCenterPose = new Pose3d();
    private Pose3d verticalStageTwoLowerBoundPose = new Pose3d();
    private Pose3d verticalStageTwoUpperBoundPose = new Pose3d();

    // Horizontal Elevator Motor Sims
    private final ElevatorSim horizontalElevatorSim;
    private final CANcoder horizontalElevatorEncoder;
    private final TitanSparkMAX horizontalElevatorMotor;
    private final RevSparkMAXSim horizontalElevatorSimMotor;

    private Pose3d horizontalElevatorRoot = new Pose3d();

    // Horizontal Elevator Stage One
    private double horizontalStageOneExtension = 0;
    private Pose3d horizontalStageOneCenterPose = new Pose3d();
    private Pose3d horizontalStageOneBackBoundPose = new Pose3d();
    private Pose3d horizontalStageOneFrontBoundPose = new Pose3d();

    // Horizontal Elevator Stage Two
    private double horizontalStageTwoExtension = 0;
    private Pose3d horizontalStageTwoCenterPose = new Pose3d();
    private Pose3d horizontalStageTwoBackBoundPose = new Pose3d();
    private Pose3d horizontalStageTwoFrontBoundPose = new Pose3d();

    public ElevatorSimSolver(
            final TalonFX verticalElevatorMotor,
            final TalonFX verticalElevatorMotorFollower,
            final CANcoder verticalElevatorEncoder,
            final CANcoder horizontalElevatorEncoder,
            final TitanSparkMAX horizontalElevatorMotor
    ) {
        this.horizontalElevatorMotor = horizontalElevatorMotor;

        // Elevator root position
        this.elevatorRootPose = Constants.Sim.Elevator.Vertical.ROBOT_TO_ROOT_MOUNT_POSE;

        final DCMotor verticalElevatorDCMotors = DCMotor.getFalcon500(2);
        this.verticalElevatorSim = new ElevatorSim(
                verticalElevatorDCMotors,
                Constants.Sim.Elevator.Vertical.GEARING,
                Constants.Sim.Elevator.Vertical.MOVING_MASS_KG,
                Constants.Sim.Elevator.Vertical.SPROCKET_RADIUS_M,
                Constants.Sim.Elevator.Vertical.MIN_TOTAL_EXT_M,
                Constants.Sim.Elevator.Vertical.MAX_TOTAL_EXT_M,
                Constants.Sim.Elevator.Vertical.SIMULATE_GRAVITY
        );

        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorSimMotors = new CTREPhoenix6TalonFXSim(
                List.of(verticalElevatorMotor, verticalElevatorMotorFollower),
                Constants.Sim.Elevator.Vertical.GEARING,
                new DCMotorSim(
                        verticalElevatorDCMotors,
                        Constants.Sim.Elevator.Vertical.GEARING,
                        Constants.Sim.Elevator.Vertical.EXT_MOI
                )
        );
        this.verticalElevatorSimMotors.attachFeedbackSensor(new SimPhoenix6CANCoder(verticalElevatorEncoder));

        final DCMotor horizontalElevatorDCMotor = DCMotor.getNEO(1);
        this.horizontalElevatorSim = new ElevatorSim(
                horizontalElevatorDCMotor,
                Constants.Sim.Elevator.Horizontal.GEARING,
                Constants.Sim.Elevator.Horizontal.MOVING_MASS_KG,
                Constants.Sim.Elevator.Horizontal.SPROCKET_RADIUS_M,
                Constants.Sim.Elevator.Horizontal.MIN_TOTAL_EXT_M,
                Constants.Sim.Elevator.Horizontal.MAX_TOTAL_EXT_M,
                Constants.Sim.Elevator.Horizontal.SIMULATE_GRAVITY
        );
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.horizontalElevatorSimMotor = new RevSparkMAXSim(
                horizontalElevatorMotor,
                horizontalElevatorDCMotor,
                new DCMotorSim(
                        horizontalElevatorDCMotor,
                        Constants.Sim.Elevator.Horizontal.GEARING,
                        Constants.Sim.Elevator.Horizontal.EXT_MOI
                )
        );
        this.horizontalElevatorSimMotor.attachFeedbackSensor(new SimPhoenix6CANCoder(horizontalElevatorEncoder));
    }

    public void updateVerticalElevatorPoses() {
        // Vertical Elevator stage one
        this.verticalStageOneLowerBoundPose = PoseUtils.withAxisOffset(
                elevatorRootPose,
                PoseUtils.Axis.Z,
                verticalStageOneExtension + Units.inchesToMeters(1.5)
        );

        this.verticalStageOneUpperBoundPose = PoseUtils.withAxisOffset(
                verticalStageOneLowerBoundPose,
                PoseUtils.Axis.Z,
                Constants.Sim.Elevator.Vertical.STAGE_ONE_HEIGHT
        );

        this.verticalStageOneCenterPose = new Pose3d(
                new Translation3d(
                        verticalStageOneLowerBoundPose.getX(),
                        verticalStageOneLowerBoundPose.getY(),
                        0.5 * (verticalStageOneLowerBoundPose.getZ() + verticalStageOneUpperBoundPose.getZ())
                ),
                verticalStageOneLowerBoundPose.getRotation()
        );

        // Vertical Elevator stage two
        this.verticalStageTwoLowerBoundPose = PoseUtils.withAxisOffset(
                verticalStageOneLowerBoundPose,
                PoseUtils.Axis.Z,
                verticalStageTwoExtension
        );

        this.verticalStageTwoUpperBoundPose = PoseUtils.withAxisOffset(
                verticalStageTwoLowerBoundPose,
                PoseUtils.Axis.Z,
                Constants.Sim.Elevator.Vertical.STAGE_TWO_HEIGHT
        );

        this.verticalStageTwoCenterPose = new Pose3d(
                new Translation3d(
                        verticalStageTwoLowerBoundPose.getX(),
                        verticalStageTwoLowerBoundPose.getY(),
                        0.5 * (verticalStageTwoLowerBoundPose.getZ() + verticalStageTwoUpperBoundPose.getZ())
                ),
                verticalStageTwoLowerBoundPose.getRotation()
        );
    }

    public void updateHorizontalElevatorPoses() {
        this.horizontalElevatorRoot = new Pose3d(
                verticalStageTwoCenterPose.getTranslation(),
                verticalStageTwoCenterPose.getRotation()
        );

        // Horizontal Stage One
        this.horizontalStageOneBackBoundPose = PoseUtils.withAxisOffset(
                horizontalElevatorRoot,
                PoseUtils.Axis.X,
                horizontalStageOneExtension
        );

        this.horizontalStageOneFrontBoundPose = PoseUtils.withAxisOffset(
                horizontalStageOneBackBoundPose,
                PoseUtils.Axis.X,
                Constants.Sim.Elevator.Horizontal.STAGE_ONE_LENGTH
        );

        this.horizontalStageOneCenterPose = new Pose3d(
                new Translation3d(
                        0.5 * (horizontalStageOneBackBoundPose.getX() + horizontalStageOneFrontBoundPose.getX()),
                        horizontalStageOneBackBoundPose.getY(),
                        horizontalStageOneBackBoundPose.getZ()
                ),
                horizontalStageOneBackBoundPose.getRotation()
        );

        // Horizontal Stage Two
        this.horizontalStageTwoBackBoundPose = PoseUtils.withAxisOffset(
                horizontalStageOneBackBoundPose,
                PoseUtils.Axis.X,
                horizontalStageTwoExtension
        );

        this.horizontalStageTwoFrontBoundPose = PoseUtils.withAxisOffset(
                horizontalStageTwoBackBoundPose,
                PoseUtils.Axis.X,
                Constants.Sim.Elevator.Horizontal.STAGE_TWO_LENGTH
        );

        this.horizontalStageTwoCenterPose = new Pose3d(
                new Translation3d(
                        0.5 * (horizontalStageTwoBackBoundPose.getX() + horizontalStageTwoFrontBoundPose.getX()),
                        horizontalStageTwoBackBoundPose.getY(),
                        horizontalStageTwoBackBoundPose.getZ()
                ),
                horizontalStageTwoBackBoundPose.getRotation()
        );
    }

    private double fromVerticalOutputRotationsToLinearDistanceMeters(final double outputRotations) {
        return outputRotations * Constants.Sim.Elevator.Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private double fromVerticalLinearDistanceMetersToOutputRotations(final double linearDistance) {
        return linearDistance / Constants.Sim.Elevator.Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private double fromHorizontalOutputRotationsToLinearDistanceMeters(final double outputRotations) {
        return outputRotations * Constants.Sim.Elevator.Horizontal.SPROCKET_CIRCUMFERENCE_M;
    }

    private double fromHorizontalLinearDistanceMetersToOutputRotations(final double linearDistance) {
        return linearDistance / Constants.Sim.Elevator.Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private double cascadeFromPreviousStage(
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

    private void updateVerticalExtension(final double verticalOutputPositionRots) {
        final double stageOneExtension = fromVerticalOutputRotationsToLinearDistanceMeters(verticalOutputPositionRots);
        final double stageTwoExtension = cascadeFromPreviousStage(
                stageOneExtension,
                Constants.Sim.Elevator.Vertical.STAGE_ONE_OFFSET,
                Constants.Sim.Elevator.Vertical.STAGE_ONE_EXT_HEIGHT,
                Constants.Sim.Elevator.Vertical.STAGE_TWO_OFFSET,
                Constants.Sim.Elevator.Vertical.STAGE_TWO_EXT_HEIGHT
        );

        verticalStageOneExtension = stageOneExtension;
        verticalStageTwoExtension = stageTwoExtension;
        updateVerticalElevatorPoses();
    }

    private void updateHorizontalExtension(final double horizontalOutputPositionRots) {
        final double stageOneExtension = fromHorizontalOutputRotationsToLinearDistanceMeters(
                horizontalOutputPositionRots
        );
        final double stageTwoExtension = cascadeFromPreviousStage(
                stageOneExtension,
                Constants.Sim.Elevator.Horizontal.STAGE_ONE_OFFSET,
                Constants.Sim.Elevator.Horizontal.STAGE_ONE_EXT_LENGTH,
                Constants.Sim.Elevator.Horizontal.STAGE_TWO_OFFSET,
                Constants.Sim.Elevator.Horizontal.STAGE_TWO_EXT_LENGTH
        );

        horizontalStageOneExtension = stageOneExtension;
        horizontalStageTwoExtension = stageTwoExtension;
        updateHorizontalElevatorPoses();
    }

    public double getVerticalElevatorPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                verticalElevatorEncoder.getPosition(),
                verticalElevatorEncoder.getVelocity()
        );
    }

    public double getHorizontalElevatorPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                horizontalElevatorEncoder.getPosition(),
                horizontalElevatorEncoder.getVelocity()
        );
    }

    private void updateVerticalInternal(final double dt) {
        verticalElevatorSimMotors.rawUpdate(
                fromVerticalLinearDistanceMetersToOutputRotations(verticalElevatorSim.getPositionMeters()),
                fromVerticalLinearDistanceMetersToOutputRotations(verticalElevatorSim.getVelocityMetersPerSecond())
        );

        verticalElevatorSim.setInputVoltage(verticalElevatorSimMotors.getMotorVoltage());
        verticalElevatorSim.update(dt);

        updateVerticalExtension(getVerticalElevatorPosition());
    }

    private void updateHorizontalInternal(final double dt) {
        horizontalElevatorSimMotor.rawUpdate(
                fromHorizontalLinearDistanceMetersToOutputRotations(horizontalElevatorSim.getPositionMeters()),
                fromHorizontalLinearDistanceMetersToOutputRotations(horizontalElevatorSim.getVelocityMetersPerSecond())
        );

        final double motorVoltage = horizontalElevatorSimMotor.getMotorVoltage();
        horizontalElevatorSim.setInputVoltage(motorVoltage);
        horizontalElevatorSim.update(dt);

        updateHorizontalExtension(getHorizontalElevatorPosition());
    }

    public void updateVertical(final double dt) {
        updateVerticalInternal(dt);
    }

    public void updateHorizontal(final double dt) {
        updateHorizontalInternal(dt);
    }

    public void update(final double dt) {
        updateVertical(dt);
        updateHorizontal(dt);
    }

    public CTREPhoenix6TalonFXSim getVerticalElevatorSimMotors() {
        return verticalElevatorSimMotors;
    }

    public ElevatorSimSolver.ElevatorSimState getElevatorSimState() {
        return new ElevatorSimSolver.ElevatorSimState(
                elevatorRootPose,
                verticalStageOneLowerBoundPose,
                verticalStageOneCenterPose,
                verticalStageOneUpperBoundPose,
                verticalStageTwoLowerBoundPose,
                verticalStageTwoCenterPose,
                verticalStageTwoUpperBoundPose,
                horizontalElevatorRoot,
                horizontalStageOneCenterPose,
                horizontalStageOneBackBoundPose,
                horizontalStageOneFrontBoundPose,
                horizontalStageTwoCenterPose,
                horizontalStageTwoBackBoundPose,
                horizontalStageTwoFrontBoundPose
        );
    }

    public record ElevatorSimState(
            Pose3d elevatorRootPose,
            Pose3d verticalStageOneLowerBoundPose,
            Pose3d verticalStageOneCenterPose,
            Pose3d verticalStageOneUpperBoundPose,
            Pose3d verticalStageTwoLowerBoundPose,
            Pose3d verticalStageTwoCenterPose,
            Pose3d verticalStageTwoUpperBoundPose,
            Pose3d horizontalRootPose,
            Pose3d horizontalStageOneCenterPose,
            Pose3d horizontalStageOneBackBoundPose,
            Pose3d horizontalStageOneFrontBoundPose,
            Pose3d horizontalStageTwoCenterPose,
            Pose3d horizontalStageTwoBackBoundPose,
            Pose3d horizontalStageTwoFrontBoundPose
    ) {
        /**
         * Log all information contained in this {@link ElevatorSimSolver.ElevatorSimState}
         * @param logKey the root logging key (with no suffixed "/")
         */
        public void log(final String logKey) {
            Logger.getInstance()
                    .recordOutput(logKey + "/ElevatorRootPose", elevatorRootPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/VerticalStageOneLowerBoundPose", verticalStageOneLowerBoundPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/VerticalStageOneCenterPose", verticalStageOneCenterPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/VerticalStageOneUpperBoundPose", verticalStageOneUpperBoundPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/VerticalStageTwoLowerBoundPose", verticalStageTwoLowerBoundPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/VerticalStageTwoCenterPose", verticalStageTwoCenterPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/VerticalStageTwoUpperBoundPose", verticalStageTwoUpperBoundPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/HorizontalRootPose", horizontalRootPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/HorizontalStageOneCenterPose", horizontalStageOneCenterPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/HorizontalStageOneBackBoundPose", horizontalStageOneBackBoundPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/HorizontalStageOneFrontBoundPose", horizontalStageOneFrontBoundPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/HorizontalStageTwoCenterPose", horizontalStageTwoCenterPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/HorizontalStageTwoBackBoundPose", horizontalStageTwoBackBoundPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/HorizontalStageTwoFrontBoundPose", horizontalStageTwoFrontBoundPose);
        }
    }
}
