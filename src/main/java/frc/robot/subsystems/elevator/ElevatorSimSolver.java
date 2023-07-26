package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.CTREPhoenix6TalonFXSim;
import frc.robot.utils.sim.RevSparkMAXSim;
import frc.robot.utils.sim.state.Computed;
import frc.robot.utils.sim.state.State;
import frc.robot.utils.sim.state.Value;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

import java.util.List;

import static frc.robot.Constants.Sim.Elevator.Horizontal;
import static frc.robot.Constants.Sim.Elevator.Vertical;
import static frc.robot.utils.PoseUtils.Axis;

public class ElevatorSimSolver {
    private final Value<Pose3d> elevatorRootPose;

    // Vertical Elevator Motor Sims
    private final ElevatorSim verticalElevatorSim;
    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final CANcoder verticalElevatorEncoder;
    private final CTREPhoenix6TalonFXSim verticalElevatorSimMotors;

    // Vertical Elevator Stage One
    private final Value<Double> verticalStageOneExtension;
    private final Computed<Pose3d> verticalStageOneLowerBoundPose;
    private final Computed<Pose3d> verticalStageOneCenterPose;
    private final Computed<Pose3d> verticalStageOneUpperBoundPose;

    // Vertical Elevator Stage Two
    private final Value<Double> verticalStageTwoExtension;
    private final Computed<Pose3d> verticalStageTwoCenterPose;
    private final Computed<Pose3d> verticalStageTwoLowerBoundPose;
    private final Computed<Pose3d> verticalStageTwoUpperBoundPose;

    // Horizontal Elevator Motor Sims
    private final ElevatorSim horizontalElevatorSim;
    private final CANcoder horizontalElevatorEncoder;
    private final TitanSparkMAX horizontalElevatorMotor;
    private final RevSparkMAXSim horizontalElevatorSimMotor;

    private final Computed<Pose3d> horizontalElevatorRoot;

    // Horizontal Elevator Stage One
    private final Value<Double> horizontalStageOneExtension;
    private final Computed<Pose3d> horizontalStageOneCenterPose;
    private final Computed<Pose3d> horizontalStageOneBackBoundPose;
    private final Computed<Pose3d> horizontalStageOneFrontBoundPose;

    // Horizontal Elevator Stage Two
    private final Value<Double> horizontalStageTwoExtension;
    private final Computed<Pose3d> horizontalStageTwoCenterPose;
    private final Computed<Pose3d> horizontalStageTwoBackBoundPose;
    private final Computed<Pose3d> horizontalStageTwoFrontBoundPose;

    public ElevatorSimSolver(
            final TalonFX verticalElevatorMotor,
            final TalonFX verticalElevatorMotorFollower,
            final CANcoder verticalElevatorEncoder,
            final CANcoder horizontalElevatorEncoder,
            final TitanSparkMAX horizontalElevatorMotor
    ) {
        this.horizontalElevatorMotor = horizontalElevatorMotor;

        // Elevator root position
        this.elevatorRootPose = new Value<>(Vertical.ROBOT_TO_ROOT_MOUNT_POSE);

        final DCMotor verticalElevatorDCMotors = DCMotor.getFalcon500(2);
        this.verticalElevatorSim = new ElevatorSim(
                verticalElevatorDCMotors,
                Vertical.GEARING,
                Vertical.MOVING_MASS_KG,
                Vertical.SPROCKET_RADIUS_M,
                Vertical.MIN_TOTAL_EXT_M,
                Vertical.MAX_TOTAL_EXT_M,
                Vertical.SIMULATE_GRAVITY
        );

        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorSimMotors = new CTREPhoenix6TalonFXSim(
                List.of(verticalElevatorMotor, verticalElevatorMotorFollower),
                Vertical.GEARING,
                new DCMotorSim(
                        verticalElevatorDCMotors,
                        Vertical.GEARING,
                        Vertical.EXT_MOI
                )
        );
        this.verticalElevatorSimMotors.attachRemoteSensor(verticalElevatorEncoder);

        final DCMotor horizontalElevatorDCMotor = DCMotor.getNEO(1);
        this.horizontalElevatorSim = new ElevatorSim(
                horizontalElevatorDCMotor,
                Horizontal.GEARING,
                Horizontal.MOVING_MASS_KG,
                Horizontal.SPROCKET_RADIUS_M,
                Horizontal.MIN_TOTAL_EXT_M,
                Horizontal.MAX_TOTAL_EXT_M,
                Horizontal.SIMULATE_GRAVITY
        );
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;
        this.horizontalElevatorSimMotor = new RevSparkMAXSim(
                horizontalElevatorMotor,
                horizontalElevatorDCMotor,
                new DCMotorSim(
                        horizontalElevatorDCMotor,
                        Horizontal.GEARING,
                        Horizontal.EXT_MOI
                )
        );
        this.horizontalElevatorSimMotor.attachRemoteSensor(horizontalElevatorEncoder);

        // Vertical Elevator stage one
        this.verticalStageOneExtension = new Value<>(0d);
        this.verticalStageOneLowerBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        elevatorRootPose.get(),
                        Axis.Z,
                        verticalStageOneExtension.get() + Units.inchesToMeters(1.5)
                ),
                elevatorRootPose,
                verticalStageOneExtension
        );

        this.verticalStageOneUpperBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        verticalStageOneLowerBoundPose.get(),
                        Axis.Z,
                        Vertical.STAGE_ONE_HEIGHT
                ),
                verticalStageOneLowerBoundPose
        );

        this.verticalStageOneCenterPose = new Computed<>(
                () -> verticalStageOneLowerBoundPose.get().interpolate(verticalStageOneUpperBoundPose.get(), 0.5),
                verticalStageOneLowerBoundPose,
                verticalStageOneUpperBoundPose
        );

        // Vertical Elevator stage two
        this.verticalStageTwoExtension = new Value<>(0d);
        this.verticalStageTwoLowerBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        verticalStageOneLowerBoundPose.get(),
                        Axis.Z,
                        verticalStageTwoExtension.get()
                ),
                verticalStageOneLowerBoundPose,
                verticalStageTwoExtension
        );

        this.verticalStageTwoUpperBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        verticalStageTwoLowerBoundPose.get(),
                        Axis.Z,
                        Vertical.STAGE_TWO_HEIGHT
                ),
                verticalStageTwoLowerBoundPose
        );

        this.verticalStageTwoCenterPose = new Computed<>(
                () -> verticalStageTwoLowerBoundPose.get().interpolate(verticalStageTwoUpperBoundPose.get(), 0.5),
                verticalStageTwoLowerBoundPose,
                verticalStageTwoUpperBoundPose
        );

        // Horizontal Elevator
        this.horizontalElevatorRoot = new Computed<>(
                () -> new Pose3d(
                        verticalStageTwoCenterPose.get().getTranslation(),
                        verticalStageTwoCenterPose.get().getRotation()
                ),
                verticalStageTwoCenterPose
        );

        // Horizontal Stage One
        this.horizontalStageOneExtension = new Value<>(0d);
        this.horizontalStageOneBackBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        horizontalElevatorRoot.get(),
                        Axis.X,
                        horizontalStageOneExtension.get()
                ),
                horizontalElevatorRoot,
                horizontalStageOneExtension
        );

        this.horizontalStageOneFrontBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        horizontalStageOneBackBoundPose.get(),
                        Axis.X,
                        Horizontal.STAGE_ONE_LENGTH
                ),
                horizontalStageOneBackBoundPose
        );

        this.horizontalStageOneCenterPose = new Computed<>(
                () -> horizontalStageOneBackBoundPose.get()
                        .interpolate(horizontalStageOneFrontBoundPose.get(), 0.5),
                horizontalStageOneBackBoundPose,
                horizontalStageOneFrontBoundPose
        );

        // Horizontal Stage Two
        this.horizontalStageTwoExtension = new Value<>(0d);
        this.horizontalStageTwoBackBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        horizontalStageOneBackBoundPose.get(),
                        Axis.X,
                        horizontalStageTwoExtension.get()
                ),
                horizontalStageOneBackBoundPose,
                horizontalStageTwoExtension
        );

        this.horizontalStageTwoFrontBoundPose = new Computed<>(
                () -> PoseUtils.withAxisOffset(
                        horizontalStageTwoBackBoundPose.get(),
                        Axis.X,
                        Horizontal.STAGE_TWO_LENGTH
                ),
                horizontalStageTwoBackBoundPose
        );

        this.horizontalStageTwoCenterPose = new Computed<>(
                () -> horizontalStageTwoBackBoundPose.get()
                        .interpolate(horizontalStageTwoFrontBoundPose.get(), 0.5),
                horizontalStageTwoBackBoundPose,
                horizontalStageTwoFrontBoundPose
        );
    }

    private double fromVerticalOutputRotationsToLinearDistanceMeters(final double outputRotations) {
        return outputRotations * Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private double fromVerticalLinearDistanceMetersToOutputRotations(final double linearDistance) {
        return linearDistance / Vertical.SPROCKET_CIRCUMFERENCE_M;
    }

    private double fromHorizontalOutputRotationsToLinearDistanceMeters(final double outputRotations) {
        return outputRotations * Horizontal.SPROCKET_CIRCUMFERENCE_M;
    }

    private double fromHorizontalLinearDistanceMetersToOutputRotations(final double linearDistance) {
        return linearDistance / Vertical.SPROCKET_CIRCUMFERENCE_M;
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

    private void updateVerticalPoses(final double verticalOutputPositionRots) {
        final double stageOneExtension = fromVerticalOutputRotationsToLinearDistanceMeters(verticalOutputPositionRots);
        final double stageTwoExtension = cascadeFromPreviousStage(
                stageOneExtension,
                Vertical.STAGE_ONE_OFFSET,
                Vertical.STAGE_ONE_EXT_HEIGHT,
                Vertical.STAGE_TWO_OFFSET,
                Vertical.STAGE_TWO_EXT_HEIGHT
        );

        verticalStageOneExtension.set(stageOneExtension);
        verticalStageTwoExtension.set(stageTwoExtension);
    }

    private void updateHorizontalPoses(final double horizontalOutputPositionRots) {
        final double stageOneExtension = fromHorizontalOutputRotationsToLinearDistanceMeters(
                horizontalOutputPositionRots
        );
        final double stageTwoExtension = cascadeFromPreviousStage(
                stageOneExtension,
                Horizontal.STAGE_ONE_OFFSET,
                Horizontal.STAGE_ONE_EXT_LENGTH,
                Horizontal.STAGE_TWO_OFFSET,
                Horizontal.STAGE_TWO_EXT_LENGTH
        );

        horizontalStageOneExtension.set(stageOneExtension);
        horizontalStageTwoExtension.set(stageTwoExtension);
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

        updateVerticalPoses(getVerticalElevatorPosition());
    }

    private void updateHorizontalInternal(final double dt) {
        horizontalElevatorSimMotor.rawUpdate(
                fromHorizontalLinearDistanceMetersToOutputRotations(horizontalElevatorSim.getPositionMeters()),
                fromHorizontalLinearDistanceMetersToOutputRotations(horizontalElevatorSim.getVelocityMetersPerSecond())
        );
//        horizontalElevatorSimMotor.update(dt);

        final double motorVoltage = horizontalElevatorSimMotor.getMotorVoltage(CANSparkMax.ControlType.kVoltage);
        Logger.getInstance().recordOutput("HorizontalMotorVoltage", motorVoltage);
        Logger.getInstance().recordOutput("HorizontalBusVoltage", horizontalElevatorMotor.getBusVoltage());
        Logger.getInstance().recordOutput("HorizontalMotorAppliedOutput", horizontalElevatorMotor.getAppliedOutput());

        horizontalElevatorSim.setInputVoltage(motorVoltage);
        horizontalElevatorSim.update(dt);

        updateHorizontalPoses(getHorizontalElevatorPosition());
    }

    public void updateVertical(final double dt) {
        Logger.getInstance().recordOutput("VerticalStageOneExtension", verticalStageOneExtension.get());
        Logger.getInstance().recordOutput("VerticalStageTwoExtension", verticalStageTwoExtension.get());

        Logger.getInstance().recordOutput("VerticalClosedLoopError", verticalElevatorMotor.getClosedLoopError().refresh().getValue());
        Logger.getInstance().recordOutput("VerticalClosedLoopReference", verticalElevatorMotor.getClosedLoopReference().refresh().getValue());

        Logger.getInstance().recordOutput("VerticalElevatorSimPosition", verticalElevatorSim.getPositionMeters());
        Logger.getInstance().recordOutput("VerticalElevatorSimVelocity", verticalElevatorSim.getVelocityMetersPerSecond());
        Logger.getInstance().recordOutput("VerticalComputedEncoderPosition", getVerticalElevatorPosition());
        Logger.getInstance().recordOutput("VerticalComputedEncoderVelocity", verticalElevatorEncoder.getVelocity().refresh().getValue());
        Logger.getInstance().recordOutput("VerticalComputedPosition", fromVerticalOutputRotationsToLinearDistanceMeters(getVerticalElevatorPosition()));

        updateVerticalInternal(dt);
    }

    public void updateHorizontal(final double dt) {
        Logger.getInstance().recordOutput("HorizontalStageOneExtension", horizontalStageOneExtension.get());
        Logger.getInstance().recordOutput("HorizontalStageTwoExtension", horizontalStageTwoExtension.get());

        Logger.getInstance().recordOutput("HorizontalElevatorSimPosition", horizontalElevatorSim.getPositionMeters());
        Logger.getInstance().recordOutput("HorizontalElevatorSimVelocity", horizontalElevatorSim.getVelocityMetersPerSecond());
        Logger.getInstance().recordOutput("HorizontalComputedEncoderPosition", getHorizontalElevatorPosition());
        Logger.getInstance().recordOutput("HorizontalComputedEncoderVelocity", horizontalElevatorEncoder.getVelocity().refresh().getValue());
        Logger.getInstance().recordOutput("HorizontalComputedPosition", fromVerticalOutputRotationsToLinearDistanceMeters(getHorizontalElevatorPosition()));

        updateHorizontalInternal(dt);
    }

    public void update(final double dt) {
        updateVertical(dt);
        updateHorizontal(dt);
    }

    public CTREPhoenix6TalonFXSim getVerticalElevatorSimMotors() {
        return verticalElevatorSimMotors;
    }

    public ElevatorSimState getElevatorSimState() {
        return new ElevatorSimState(
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
        public ElevatorSimState(
                final State<Pose3d> elevatorRootPose,
                final State<Pose3d> verticalStageOneLowerBoundPose,
                final State<Pose3d> verticalStageOneCenterPose,
                final State<Pose3d> verticalStageOneUpperBoundPose,
                final State<Pose3d> verticalStageTwoLowerBoundPose,
                final State<Pose3d> verticalStageTwoCenterPose,
                final State<Pose3d> verticalStageTwoUpperBoundPose,
                final State<Pose3d> horizontalRootPose,
                final State<Pose3d> horizontalStageOneCenterPose,
                final State<Pose3d> horizontalStageOneBackBoundPose,
                final State<Pose3d> horizontalStageOneFrontBoundPose,
                final State<Pose3d> horizontalStageTwoCenterPose,
                final State<Pose3d> horizontalStageTwoBackBoundPose,
                final State<Pose3d> horizontalStageTwoFrontBoundPose
        ) {
            this(
                    elevatorRootPose.get(),
                    verticalStageOneLowerBoundPose.get(),
                    verticalStageOneCenterPose.get(),
                    verticalStageOneUpperBoundPose.get(),
                    verticalStageTwoLowerBoundPose.get(),
                    verticalStageTwoCenterPose.get(),
                    verticalStageTwoUpperBoundPose.get(),
                    horizontalRootPose.get(),
                    horizontalStageOneCenterPose.get(),
                    horizontalStageOneBackBoundPose.get(),
                    horizontalStageOneFrontBoundPose.get(),
                    horizontalStageTwoCenterPose.get(),
                    horizontalStageTwoBackBoundPose.get(),
                    horizontalStageTwoFrontBoundPose.get()
            );
        }

        public void log(final String root) {
            Logger.getInstance()
                    .recordOutput(root + "/ElevatorRootPose", elevatorRootPose);
            Logger.getInstance()
                    .recordOutput(root + "/VerticalStageOneLowerBoundPose", verticalStageOneLowerBoundPose);
            Logger.getInstance()
                    .recordOutput(root + "/VerticalStageOneCenterPose", verticalStageOneCenterPose);
            Logger.getInstance()
                    .recordOutput(root + "/VerticalStageOneUpperBoundPose", verticalStageOneUpperBoundPose);
            Logger.getInstance()
                    .recordOutput(root + "/VerticalStageTwoLowerBoundPose", verticalStageTwoLowerBoundPose);
            Logger.getInstance()
                    .recordOutput(root + "/VerticalStageTwoCenterPose", verticalStageTwoCenterPose);
            Logger.getInstance()
                    .recordOutput(root + "/VerticalStageTwoUpperBoundPose", verticalStageTwoUpperBoundPose);
            Logger.getInstance()
                    .recordOutput(root + "/HorizontalRootPose", horizontalRootPose);
            Logger.getInstance()
                    .recordOutput(root + "/HorizontalStageOneCenterPose", horizontalStageOneCenterPose);
            Logger.getInstance()
                    .recordOutput(root + "/HorizontalStageOneBackBoundPose", horizontalStageOneBackBoundPose);
            Logger.getInstance()
                    .recordOutput(root + "/HorizontalStageOneFrontBoundPose", horizontalStageOneFrontBoundPose);
            Logger.getInstance()
                    .recordOutput(root + "/HorizontalStageTwoCenterPose", horizontalStageTwoCenterPose);
            Logger.getInstance()
                    .recordOutput(root + "/HorizontalStageTwoBackBoundPose", horizontalStageTwoBackBoundPose);
            Logger.getInstance()
                    .recordOutput(root + "/HorizontalStageTwoFrontBoundPose", horizontalStageTwoFrontBoundPose);
        }
    }
}
