package frc.robot.utils.sim.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.rev.RevUtils;
import frc.robot.utils.sim.feedback.SimFeedbackSensor;
import frc.robot.wrappers.motors.TitanSparkMAX;

import java.util.List;

public class RevSparkMAXSim implements SimMotorController {
    private final List<TitanSparkMAX> sparkMaxes;
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final boolean isSingularSparkMax;

    private boolean hasRemoteSensor = false;
    private SimFeedbackSensor feedbackSensor;

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

    @Override
    public void attachFeedbackSensor(final SimFeedbackSensor feedbackSensor) {
        if (hasRemoteSensor) {
            throw new RuntimeException("Attempt to attach SimFeedbackSensor when one is already attached!");
        }

        this.hasRemoteSensor = true;
        this.feedbackSensor = feedbackSensor;
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
        final double dtMs = Units.secondsToMilliseconds(dt);
        for (final TitanSparkMAX sparkMax : sparkMaxes) {
            final RelativeEncoder relativeEncoder = sparkMax.getEncoder();

            final double position = relativeEncoder.getPosition();
            final double velocity = relativeEncoder.getVelocity();
            final double positionConversionFactor = relativeEncoder.getPositionConversionFactor();

            relativeEncoder.setPosition(position + velocity * dtMs / 60000.0 * positionConversionFactor);
        }
    }

    @Override
    public void update(final double dt) {
        updateSparkMaxesInternal(dt);

        final double motorVoltage = getMotorVoltage();
        dcMotorSim.setInputVoltage(motorVoltage);
        dcMotorSim.update(dt);

        final double mechanismAngularPositionRots = getAngularPositionRots();
        final double mechanismAngularVelocityRotsPerSec = getAngularVelocityRotsPerSec();

        if (hasRemoteSensor) {
            feedbackSensor.setRawPosition(mechanismAngularPositionRots);
            feedbackSensor.setVelocity(mechanismAngularVelocityRotsPerSec);
        }
    }

    @Override
    public void rawUpdate(final double mechanismPositionRots, final double mechanismVelocityRotsPerSec) {
        updateSparkMaxesRawInternal(mechanismPositionRots);

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

    /**
     * Get the output voltage of the motor(s)
     * @param controlType the {@link com.revrobotics.CANSparkMax.ControlType} of the SparkMAX(s)
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

    @Override
    public double getMotorVoltage() {
        return getMotorVoltage(CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public double getMotorCurrent() {
        return dcMotorSim.getCurrentDrawAmps();
    }
}
