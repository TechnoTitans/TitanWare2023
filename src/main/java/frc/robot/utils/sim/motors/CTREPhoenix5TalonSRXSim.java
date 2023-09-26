package frc.robot.utils.sim.motors;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.ctre.Phoenix5Utils;
import frc.robot.utils.sim.feedback.SimFeedbackSensor;

import java.util.List;

public class CTREPhoenix5TalonSRXSim implements SimMotorController {
    private final List<TalonSRXSimCollection> simCollections;
    private final DCMotorSim dcMotorSim;
    private final double gearRatio;
    private final boolean isSingularTalonSRX;

    private boolean hasRemoteSensor = false;
    private SimFeedbackSensor feedbackSensor;

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

    @Override
    public void attachFeedbackSensor(final SimFeedbackSensor feedbackSensor) {
        if (hasRemoteSensor) {
            throw new RuntimeException("Attempt to attach SimFeedbackSensor when one is already attached!");
        }

        this.hasRemoteSensor = true;
        this.feedbackSensor = feedbackSensor;
    }

    @Override
    public void update(final double dt) {
        final double batteryVoltage = RobotController.getBatteryVoltage();
        for (final TalonSRXSimCollection simCollection : simCollections) {
            simCollection.setBusVoltage(batteryVoltage);
        }

        final double averageLeadVoltage = getMotorVoltage();
        dcMotorSim.setInputVoltage(averageLeadVoltage);
        dcMotorSim.update(dt);

        final double mechanismAngularPositionRots = dcMotorSim.getAngularPositionRotations();
        final double mechanismAngularVelocityRotsPerSec = Units.radiansToRotations(
                dcMotorSim.getAngularVelocityRadPerSec()
        );

        for (final TalonSRXSimCollection simCollection : simCollections) {
            simCollection.setQuadratureRawPosition(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(gearRatio * mechanismAngularPositionRots)
            );
            simCollection.setQuadratureVelocity(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(gearRatio * mechanismAngularVelocityRotsPerSec)
            );
        }

        if (hasRemoteSensor) {
            feedbackSensor.setRawPosition(mechanismAngularPositionRots);
            feedbackSensor.setVelocity(mechanismAngularVelocityRotsPerSec);
        }
    }

    @Override
    public void rawUpdate(double mechanismPositionRots, double mechanismVelocityRotsPerSec) {
        for (final TalonSRXSimCollection simCollection : simCollections) {
            simCollection.setQuadratureRawPosition(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(gearRatio * mechanismPositionRots)
            );
            simCollection.setQuadratureVelocity(
                    Phoenix5Utils.rotationsToCTREPhoenix5NativeUnits(gearRatio * mechanismVelocityRotsPerSec)
            );
        }

        if (hasRemoteSensor) {
            feedbackSensor.setRawPosition(mechanismPositionRots);
            feedbackSensor.setVelocity(mechanismVelocityRotsPerSec);
        }
    }

    @Override
    public double getAngularPositionRots() {
        return dcMotorSim.getAngularPositionRotations();
    }

    @Override
    public double getAngularVelocityRotsPerSec() {
        return Units.radiansToRotations(dcMotorSim.getAngularVelocityRadPerSec());
    }

    @Override
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
