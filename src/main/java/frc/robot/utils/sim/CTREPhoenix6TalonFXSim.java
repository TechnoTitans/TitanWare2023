package frc.robot.utils.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class CTREPhoenix6TalonFXSim {
    private final List<TalonFXSimState> simStates;
    private final DCMotorSim dcMotorSim;
    private final double gearRatio;

    private final boolean isSingularTalonFX;

    private boolean hasRemoteSensor = false;
    private CANcoderSimState cancoderSimState;

    private CTREPhoenix6TalonFXSim(
            final List<TalonFX> talonFXControllers,
            final List<TalonFXSimState> simStates,
            final DCMotorSim dcMotorSim,
            final double gearRatio
    ) {
        if (talonFXControllers.isEmpty() || simStates.isEmpty()) {
            throw new IllegalArgumentException("TalonFX must not be empty! TalonFXSimStates must not be empty!");
        }

        this.simStates = simStates;
        this.dcMotorSim = dcMotorSim;
        this.gearRatio = gearRatio;

        this.isSingularTalonFX = talonFXControllers.size() == 1 && simStates.size() == 1;
    }

    public CTREPhoenix6TalonFXSim(
            final List<TalonFX> talonFXControllers,
            final double gearRatio,
            final DCMotorSim motorSim
    ) {
        this(talonFXControllers, talonFXControllers.stream().map(TalonFX::getSimState).toList(), motorSim, gearRatio);
    }

    public CTREPhoenix6TalonFXSim(final TalonFX talonSRX, final double gearRatio, final DCMotorSim motorSim) {
        this(List.of(talonSRX), gearRatio, motorSim);
    }

    public void attachRemoteSensor(final CANcoder canCoder) {
        if (hasRemoteSensor) {
            throw new RuntimeException("attempt to attach remote sensor when one is already attached!");
        }

        this.hasRemoteSensor = true;
        this.cancoderSimState = canCoder.getSimState();
    }

    public void update(final double dt) {
        final double motorVoltage = isSingularTalonFX
                ? simStates.get(0).getMotorVoltage()
                : simStates.stream().mapToDouble(TalonFXSimState::getMotorVoltage).average().orElseThrow();

        dcMotorSim.setInputVoltage(motorVoltage);
        dcMotorSim.update(dt);

        final double rotorAngularPositionRots = dcMotorSim.getAngularPositionRotations();
        final double rotorAngularVelocityRotsPerSec = Units.radiansToRotations(dcMotorSim.getAngularVelocityRadPerSec());

        for (final TalonFXSimState simState : simStates) {
            simState.setRawRotorPosition(rotorAngularPositionRots);
            simState.setRotorVelocity(rotorAngularVelocityRotsPerSec);
            simState.setSupplyVoltage(
                    12 - (simState.getSupplyCurrent() * Constants.Sim.FALCON_MOTOR_RESISTANCE)
            );
        }

        if (hasRemoteSensor) {
            cancoderSimState.setRawPosition(rotorAngularPositionRots);
            cancoderSimState.setVelocity(rotorAngularVelocityRotsPerSec);
        }
    }

    /**
     * Position of the simulated motor
     * @return the position, in rotations
     */
    public double getAngularPositionRots() {
        return dcMotorSim.getAngularPositionRotations();
    }

    /**
     * Velocity of the simulated motor
     * @return the velocity, in rotations/sec
     */
    public double getAngularVelocityRotsPerSec() {
        return Units.radiansToRotations(dcMotorSim.getAngularVelocityRadPerSec());
    }
}
