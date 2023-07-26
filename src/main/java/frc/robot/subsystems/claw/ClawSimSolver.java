package frc.robot.subsystems.claw;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.elevator.ElevatorSimSolver;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.CTREPhoenix5TalonSRXSim;
import frc.robot.utils.sim.RevSparkMAXSim;
import frc.robot.utils.sim.state.Computed;
import frc.robot.utils.sim.state.State;
import frc.robot.utils.sim.state.Value;
import frc.robot.wrappers.motors.TitanSRX;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Sim.Claw;

public class ClawSimSolver {
    private final TitanSRX clawMainWheelBag, clawFollowerWheelBag;
    private final CTREPhoenix5TalonSRXSim clawMainWheelSim, clawFollowerWheelSim;
    private final TitanSRX clawOpenCloseMotor;
    private final CTREPhoenix5TalonSRXSim clawOpenCloseSim;
    private final CANCoder clawOpenCloseEncoder;

    private final TitanSparkMAX clawTiltNeo;
    private final CANcoder clawTiltEncoder;
    private final SingleJointedArmSim clawTiltSim;
    private final RevSparkMAXSim clawTiltSimMotor;

    private final Value<Pose3d> clawRootPose;
    private final Value<Double> clawTiltRots;
    private final Computed<Pose3d> clawPose;

    public ClawSimSolver(
            final TitanSRX clawMainWheelBag,
            final TitanSRX clawFollowerWheelBag,
            final TitanSRX clawOpenCloseMotor,
            final CANCoder clawOpenCloseEncoder,
            final TitanSparkMAX clawTiltNeo,
            final CANcoder clawTiltEncoder
    ) {
        this.clawRootPose = new Value<>(new Pose3d().transformBy(Claw.CARRIAGE_TO_ROOT_MOUNT_TRANSFORM));

        this.clawMainWheelBag = clawMainWheelBag;
        this.clawFollowerWheelBag = clawFollowerWheelBag;

        this.clawMainWheelSim = new CTREPhoenix5TalonSRXSim(
                clawMainWheelBag,
                Claw.INTAKE_WHEELS_GEARING,
                new DCMotorSim(
                        DCMotor.getBag(1),
                        Claw.INTAKE_WHEELS_GEARING,
                        Claw.INTAKE_WHEELS_MOI
                )
        );

        this.clawFollowerWheelSim = new CTREPhoenix5TalonSRXSim(
                clawFollowerWheelBag,
                Claw.INTAKE_WHEELS_GEARING,
                new DCMotorSim(
                        DCMotor.getBag(1),
                        Claw.INTAKE_WHEELS_GEARING,
                        Claw.INTAKE_WHEELS_MOI
                )
        );

        this.clawTiltNeo = clawTiltNeo;
        this.clawTiltEncoder = clawTiltEncoder;

        final DCMotor tiltDCMotor = DCMotor.getNEO(1);
        this.clawTiltSim = new SingleJointedArmSim(
                tiltDCMotor,
                Claw.TILT_GEARING,
                Claw.TILT_MOI,
                Claw.CLAW_LENGTH_M,
                Claw.TILT_MIN_ANGLE_RAD,
                Claw.TILT_MAX_ANGLE_RAD,
                Claw.TILT_SIMULATE_GRAVITY
        );

        this.clawTiltSimMotor = new RevSparkMAXSim(
                clawTiltNeo,
                tiltDCMotor,
                new DCMotorSim(
                        tiltDCMotor,
                        Claw.TILT_GEARING,
                        Claw.TILT_MOI
                )
        );
        this.clawTiltSimMotor.attachRemoteSensor(clawTiltEncoder);

        this.clawOpenCloseMotor = clawOpenCloseMotor;
        this.clawOpenCloseEncoder = clawOpenCloseEncoder;
        this.clawOpenCloseSim = new CTREPhoenix5TalonSRXSim(
                clawOpenCloseMotor,
                Claw.OPEN_CLOSE_GEARING,
                new DCMotorSim(
                        DCMotor.getBag(1),
                        Claw.OPEN_CLOSE_GEARING,
                        Claw.OPEN_CLOSE_MOI
                )
        );
        this.clawOpenCloseSim.attachRemoteSensor(clawOpenCloseEncoder);

        this.clawTiltRots = new Value<>(0d);
        this.clawPose = new Computed<>(
                () -> clawRootPose.get()
                        .transformBy(
                                new Transform3d(
                                        Claw.SHAFT_TO_CENTER_TRANSLATION.rotateBy(new Rotation3d(
                                                VecBuilder.fill(0, 1, 0),
                                                Units.rotationsToRadians(clawTiltRots.get())
                                        )),
                                        new Rotation3d(0, Units.rotationsToRadians(clawTiltRots.get()), 0)
                                )
                        ),
                clawRootPose,
                clawTiltRots
        );
    }

    public SingleJointedArmSim getClawTiltSim() {
        return clawTiltSim;
    }

    public double getClawTiltPosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
                clawTiltEncoder.getAbsolutePosition(),
                clawTiltEncoder.getVelocity()
        );
    }

    private void updateIntakeWheelsInternal(final double dt) {
        clawMainWheelSim.update(dt);
        clawFollowerWheelSim.update(dt);
    }

    private void updateOpenCloseInternal(final double dt) {
        clawOpenCloseSim.update(dt);
    }

    private void updateTiltInternal(final double dt, final ElevatorSimSolver.ElevatorSimState elevatorSimState) {
        clawTiltSimMotor.rawUpdate(
                Units.radiansToRotations(clawTiltSim.getAngleRads()),
                Units.radiansToRotations(clawTiltSim.getVelocityRadPerSec())
        );

        clawTiltSim.setInputVoltage(clawTiltSimMotor.getMotorVoltage(CANSparkMax.ControlType.kVoltage));
        clawTiltSim.update(dt);

        clawRootPose.set(
                elevatorSimState
                        .horizontalStageTwoFrontBoundPose()
                        .transformBy(Claw.CARRIAGE_TO_ROOT_MOUNT_TRANSFORM)
        );
        clawTiltRots.set(getClawTiltPosition());
    }

    public void update(final double dt, final ElevatorSimSolver.ElevatorSimState elevatorSimState) {
        Logger.getInstance().recordOutput("ClawTiltEncoderPosition", getClawTiltPosition());
        Logger.getInstance().recordOutput("ClawTiltEncoderVelocity", clawTiltEncoder.getVelocity().refresh().getValue());

        Logger.getInstance().recordOutput("ArmSimPosition", Units.radiansToRotations(clawTiltSim.getAngleRads()));
        Logger.getInstance().recordOutput("ArmSimVelocity", Units.radiansToRotations(clawTiltSim.getVelocityRadPerSec()));

        Logger.getInstance().recordOutput("SparkMAXMotorVoltage", clawTiltSimMotor.getMotorVoltage(CANSparkMax.ControlType.kVoltage));
        Logger.getInstance().recordOutput("SparkMAXBusVoltage", clawTiltNeo.getBusVoltage());
        Logger.getInstance().recordOutput("SparkMAXAppliedOutput", clawTiltNeo.getAppliedOutput());

        updateTiltInternal(dt, elevatorSimState);
        updateOpenCloseInternal(dt);
        updateIntakeWheelsInternal(dt);
    }

    public ClawSimState getClawSimState() {
        return new ClawSimState(
                clawRootPose,
                clawPose
        );
    }

    public record ClawSimState(
            Pose3d clawRootPose,
            Pose3d clawPose
    ) {
        public ClawSimState(
                final State<Pose3d> clawRootPose,
                final State<Pose3d> clawPose
        ) {
            this(
                    clawRootPose.get(),
                    clawPose.get()
            );
        }

        public void log(final String root) {
            Logger.getInstance()
                    .recordOutput(root + "/ClawRootPose", clawRootPose);
            Logger.getInstance()
                    .recordOutput(root + "/ClawPose", clawPose);
        }
    }
}
