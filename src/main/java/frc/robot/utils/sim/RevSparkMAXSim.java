package frc.robot.utils.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.rev.RevUtils;
import frc.robot.wrappers.motors.TitanSparkMAX;

import java.util.List;

public class RevSparkMAXSim {
    private final List<TitanSparkMAX> sparkMaxes;
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final boolean isSingularSparkMax;

    private boolean hasRemoteSensor = false;
    private CANcoderSimState cancoderSimState;

    public RevSparkMAXSim(
            final List<TitanSparkMAX> sparkMaxes,
            final DCMotor dcMotor,
            final DCMotorSim dcMotorSim
    ) {
        if (sparkMaxes.isEmpty()) {
            throw new IllegalArgumentException("SparkMaxes must not be empty!");
        }

        this.sparkMaxes = sparkMaxes;
        this.dcMotor = dcMotor;
        this.dcMotorSim = dcMotorSim;

        this.isSingularSparkMax = sparkMaxes.size() == 1;

        setupSparkMaxSims();
    }

    public RevSparkMAXSim(
            final TitanSparkMAX canSparkMax,
            final DCMotor dcMotor,
            final DCMotorSim dcMotorSim
    ) {
        this(List.of(canSparkMax), dcMotor, dcMotorSim);
    }

    public void attachRemoteSensor(final CANcoder canCoder) {
        if (hasRemoteSensor) {
            throw new RuntimeException("attempt to attach remote sensor when one is already attached!");
        }

        this.hasRemoteSensor = true;
        this.cancoderSimState = canCoder.getSimState();
    }

    private void setupSparkMaxSims() {
        for (final TitanSparkMAX sparkMax : sparkMaxes) {
            final REVLibError stallTorqueErrCode = sparkMax.setSimStallTorque((float) dcMotor.stallTorqueNewtonMeters);
            final REVLibError freeSpeedErrCode = sparkMax.setSimFreeSpeed(
                    (float) Units.radiansPerSecondToRotationsPerMinute(dcMotor.freeSpeedRadPerSec)
            );

            RevUtils.reportIfNotOk(sparkMax, stallTorqueErrCode);
            RevUtils.reportIfNotOk(sparkMax, freeSpeedErrCode);
        }
    }

    private void updateSparkMaxesRawInternal(final double position) {
        for (final TitanSparkMAX sparkMax : sparkMaxes) {
            sparkMax.getEncoder().setPosition(position);
        }
    }

    /**
     * Internal method. Updates the associated SparkMAX controllers through time.
     * @param dt the amount of time since the last update call (in seconds)
     */
    private void updateSparkMaxesInternal(final double dt) {
        final double dt_ms = Units.secondsToMilliseconds(dt);
        for (final TitanSparkMAX sparkMax : sparkMaxes) {
            final RelativeEncoder relativeEncoder = sparkMax.getEncoder();

            final double position = relativeEncoder.getPosition();
            final double velocity = relativeEncoder.getVelocity();
            final double positionConversionFactor = relativeEncoder.getPositionConversionFactor();

            relativeEncoder.setPosition(position + velocity * dt_ms / 60000.0 * positionConversionFactor);
        }
    }

    /**
     * Updates SparkMAX controllers with time.
     * @param dt the amount of time since the last update call (in seconds)
     */
    public void update(final double dt) {
        final double motorVoltage = getMotorVoltage(CANSparkMax.ControlType.kVoltage);
        dcMotorSim.setInputVoltage(motorVoltage);
        dcMotorSim.update(dt);

        updateSparkMaxesInternal(dt);

        final double mechanismAngularPositionRots = getAngularPositionRots();
        final double mechanismAngularVelocityRotsPerSec = getAngularVelocityRotsPerSec();

        if (hasRemoteSensor) {
            cancoderSimState.setRawPosition(mechanismAngularPositionRots);
            cancoderSimState.setVelocity(mechanismAngularVelocityRotsPerSec);
        }
    }

    public void rawUpdate(final double mechanismPositionRots, final double mechanismVelocityRotsPerSec) {
        updateSparkMaxesRawInternal(mechanismPositionRots);

        if (hasRemoteSensor) {
            cancoderSimState.setRawPosition(mechanismPositionRots);
            cancoderSimState.setVelocity(mechanismVelocityRotsPerSec);
        }
    }

    /**
     * Position of the simulated motor (output shaft, or mechanism, position)
     * @return the position, in rotations
     */
    public double getAngularPositionRots() {
        return dcMotorSim.getAngularPositionRotations();
    }

    /**
     * Velocity of the simulated motor (output shaft, or mechanism, velocity)
     * @return the velocity, in rotations/sec
     */
    public double getAngularVelocityRotsPerSec() {
        return Units.radiansToRotations(dcMotorSim.getAngularVelocityRadPerSec());
    }

    /**
     * Get the output voltage of the motor(s)
     * @return the output voltage (in volts)
     */
    public double getMotorVoltage(final CANSparkMax.ControlType controlType) {
        if (isSingularSparkMax) {
            return RevUtils.getSparkMAXMotorVoltage(sparkMaxes.get(0), controlType);
        } else {
            return sparkMaxes.stream()
                    .mapToDouble(sparkMax -> RevUtils.getSparkMAXMotorVoltage(sparkMax, controlType))
                    .average()
                    .orElseThrow();
        }
    }
}
