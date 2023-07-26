package frc.robot.utils.sim;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.ctre.Phoenix5Utils;

import java.util.List;

public class CTREPhoenix5TalonSRXSim {
    private final List<TalonSRXSimCollection> simCollections;
    private final DCMotorSim dcMotorSim;
    private final double gearRatio;
    private final boolean isSingularTalonSRX;

    private boolean hasRemoteSensor = false;
    private CANCoderSimCollection canCoderSimCollection;

    private CTREPhoenix5TalonSRXSim(
            final List<TalonSRX> talonSRXControllers,
            final List<TalonSRXSimCollection> simCollections,
            final DCMotorSim motorSim,
            final double gearRatio
    ) {
        if (talonSRXControllers.isEmpty() || simCollections.isEmpty()) {
            throw new IllegalArgumentException("TalonSRX must not be empty! TalonSRXSimCollection must not be empty!");
        }

        this.simCollections = simCollections;
        this.dcMotorSim = motorSim;
        this.gearRatio = gearRatio;

        this.isSingularTalonSRX = talonSRXControllers.size() == 1 && simCollections.size() == 1;
    }

    public CTREPhoenix5TalonSRXSim(
            final List<TalonSRX> talonSRXControllers,
            final double gearRatio,
            final DCMotorSim motorSim
    ) {
        this(
                talonSRXControllers,
                talonSRXControllers.stream().map(TalonSRX::getSimCollection).toList(),
                motorSim,
                gearRatio
        );
    }

    public CTREPhoenix5TalonSRXSim(final TalonSRX talonSRX, final double gearRatio, final DCMotorSim motorSim) {
        this(List.of(talonSRX), gearRatio, motorSim);
    }

    public void attachRemoteSensor(final CANCoder canCoder) {
        if (hasRemoteSensor) {
            throw new RuntimeException("attempt to attach remote sensor when one is already attached!");
        }

        this.hasRemoteSensor = true;
        this.canCoderSimCollection = canCoder.getSimCollection();
    }

    public void update(final double dt) {
        final double batteryVoltage = RobotController.getBatteryVoltage();
        for (final TalonSRXSimCollection simCollection : simCollections) {
            simCollection.setBusVoltage(batteryVoltage);
        }

        final double averageLeadVoltage = getMotorVoltage();
        dcMotorSim.setInputVoltage(averageLeadVoltage);
        dcMotorSim.update(dt);

        final double mechanismAngularPositionRots = dcMotorSim.getAngularPositionRotations();
        final double angularVelocityRotsPerSec = Units.radiansToRotations(
                dcMotorSim.getAngularVelocityRadPerSec()
        );

        for (final TalonSRXSimCollection simCollection : simCollections) {
            simCollection.setQuadratureRawPosition(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(gearRatio * mechanismAngularPositionRots)
            );

            simCollection.setQuadratureVelocity(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(gearRatio * angularVelocityRotsPerSec)
            );
        }

        if (hasRemoteSensor) {
            canCoderSimCollection.setRawPosition(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(mechanismAngularPositionRots)
            );
            canCoderSimCollection.setRawPosition(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(mechanismAngularPositionRots)
            );
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

    /**
     * Get the output voltage of the motor(s)
     * @return the output voltage (in volts)
     */
    public double getMotorVoltage() {
        if (isSingularTalonSRX) {
            return simCollections.get(0).getMotorOutputLeadVoltage();
        } else {
            return simCollections.stream()
                    .mapToDouble(TalonSRXSimCollection::getMotorOutputLeadVoltage)
                    .average()
                    .orElseThrow();
        }
    }
}
