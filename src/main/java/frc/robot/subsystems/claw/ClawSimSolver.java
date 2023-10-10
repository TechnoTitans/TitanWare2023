package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
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
import frc.robot.utils.sim.feedback.SimPhoenix5CANCoder;
import frc.robot.utils.sim.feedback.SimPhoenix6CANCoder;
import frc.robot.utils.sim.motors.CTREPhoenix5TalonSRXSim;
import frc.robot.utils.sim.motors.RevSparkMAXSim;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Sim.Claw;

public class ClawSimSolver {
    private final CTREPhoenix5TalonSRXSim clawMainWheelSim, clawFollowerWheelSim;
    private final CTREPhoenix5TalonSRXSim clawOpenCloseSim;

    private final CANcoder clawTiltEncoder;
    private final SingleJointedArmSim clawTiltSim;
    private final RevSparkMAXSim clawTiltSimMotor;

    private Pose3d clawRootPose = new Pose3d().transformBy(Claw.CARRIAGE_TO_ROOT_MOUNT_TRANSFORM);
    private double clawTiltRots = 0;
    private Pose3d clawPose = new Pose3d();

    public ClawSimSolver(
            final TalonSRX clawMainWheelBag,
            final TalonSRX clawFollowerWheelBag,
            final TalonSRX clawOpenCloseMotor,
            final CANCoder clawOpenCloseEncoder,
            final TitanSparkMAX clawTiltNeo,
            final CANcoder clawTiltEncoder
    ) {
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

        final DCMotor tiltDCMotor = DCMotor.getNEO(1);
        this.clawTiltEncoder = clawTiltEncoder;
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
        this.clawTiltSimMotor.attachFeedbackSensor(new SimPhoenix6CANCoder(clawTiltEncoder));

        this.clawOpenCloseSim = new CTREPhoenix5TalonSRXSim(
                clawOpenCloseMotor,
                Claw.OPEN_CLOSE_GEARING,
                new DCMotorSim(
                        DCMotor.getBag(1),
                        Claw.OPEN_CLOSE_GEARING,
                        Claw.OPEN_CLOSE_MOI
                )
        );
        this.clawOpenCloseSim.attachFeedbackSensor(new SimPhoenix5CANCoder(clawOpenCloseEncoder));
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

    private void updatePoses() {
        this.clawPose = clawRootPose
                .transformBy(
                        new Transform3d(
                                Claw.SHAFT_TO_CENTER_TRANSLATION.rotateBy(new Rotation3d(
                                        VecBuilder.fill(0, 1, 0),
                                        Units.rotationsToRadians(clawTiltRots)
                                )),
                                new Rotation3d(0, Units.rotationsToRadians(clawTiltRots), 0)
                        )
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

        final double clawTiltMotorVoltage = clawTiltSimMotor.getMotorVoltage();
        Logger.getInstance().recordOutput("ClawTiltMotorVoltage", clawTiltMotorVoltage);
        Logger.getInstance().recordOutput("ClawTiltAngleRots", Units.radiansToRotations(clawTiltSim.getAngleRads()));
        Logger.getInstance().recordOutput(
                "ClawTiltVelocityRotsPerSec",
                Units.radiansToRotations(clawTiltSim.getVelocityRadPerSec())
        );

        clawTiltSim.setInputVoltage(clawTiltMotorVoltage);
        clawTiltSim.update(dt);

        clawRootPose =
                elevatorSimState
                        .horizontalStageTwoFrontBoundPose()
                        .transformBy(Claw.CARRIAGE_TO_ROOT_MOUNT_TRANSFORM);
        clawTiltRots = getClawTiltPosition();
    }

    public void update(final double dt, final ElevatorSimSolver.ElevatorSimState elevatorSimState) {
        updateTiltInternal(dt, elevatorSimState);
        updateOpenCloseInternal(dt);
        updateIntakeWheelsInternal(dt);

        updatePoses();
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
        public void log(final String logKey) {
            Logger.getInstance()
                    .recordOutput(logKey + "/ClawRootPose", clawRootPose);
            Logger.getInstance()
                    .recordOutput(logKey + "/ClawPose", clawPose);
        }
    }
}
