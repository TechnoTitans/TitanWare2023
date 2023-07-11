package frc.robot.utils.sim;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.List;

public class CTREPhoenix5TalonSRXSim {
    private final List<TalonSRXSimCollection> simCollections;
    private final DCMotorSim dcMotorSim;

    private CTREPhoenix5TalonSRXSim(
            final List<TalonSRX> talonSRXControllers,
            final List<TalonSRXSimCollection> simCollections,
            final DCMotorSim motorSim
    ) {
        if (talonSRXControllers.isEmpty() || simCollections.isEmpty()) {
            throw new IllegalArgumentException("TalonSRX must not be empty! TalonSRXSimCollection must not be empty!");
        }

        this.simCollections = simCollections;
        this.dcMotorSim = motorSim;
    }

    public CTREPhoenix5TalonSRXSim(final List<TalonSRX> talonSRXControllers, final DCMotorSim motorSim) {
        this(talonSRXControllers, talonSRXControllers.stream().map(TalonSRX::getSimCollection).toList(), motorSim);
    }

    public CTREPhoenix5TalonSRXSim(final TalonSRX talonSRX, final DCMotorSim motorSim) {
        this(List.of(talonSRX), motorSim);
    }

    public void update(final double dt) {
        final double batteryVoltage = RobotController.getBatteryVoltage();
        for (final TalonSRXSimCollection simCollection : simCollections) {
            simCollection.setBusVoltage(batteryVoltage);
        }

        final double averageLeadVoltage = simCollections.stream()
                .mapToDouble(TalonSRXSimCollection::getMotorOutputLeadVoltage)
                .average()
                .orElseThrow();

        dcMotorSim.setInputVoltage(averageLeadVoltage);
        dcMotorSim.update(dt);

        final double angularPositionRotations = dcMotorSim.getAngularPositionRotations();
        final double angularVelocityRotationsPerSec = Units.radiansToRotations(
                dcMotorSim.getAngularVelocityRadPerSec()
        );

        for (final TalonSRXSimCollection simCollection : simCollections) {
            simCollection.setQuadratureRawPosition(
                    SimUtils.rotationsToCTREPhoenix5NativeUnits(angularPositionRotations)
            );

            simCollection.setQuadratureVelocity(
                    SimUtils.rotationsToCTREPhoenix5NativeUnits(angularVelocityRotationsPerSec)
            );
        }
    }
}
