package frc.robot.utils.sim.feedback;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.wrappers.motors.TitanSparkMAX;
import org.littletonrobotics.junction.Logger;

public class SimSparkMaxAbsoluteEncoder implements AbsoluteEncoder, SimFeedbackSensor {
    private final TitanSparkMAX sparkMax;
    private final RelativeEncoder relativeEncoder;

    private double positionConversionFactor = 1;
    private double velocityConversionFactor = 1;

    private boolean inverted = false;

    private double zeroOffset = 0;
    private double setRawPositionOffset = 0;
    private double addedPositionOffset = 0;
    private double setVelocityOffset = 0;

    public SimSparkMaxAbsoluteEncoder(final TitanSparkMAX sparkMax, final SparkMaxAbsoluteEncoder.Type type) {
        if (type != SparkMaxAbsoluteEncoder.Type.kDutyCycle) {
            throw new IllegalStateException("NotImplemented!");
        }

        this.sparkMax = sparkMax;
        this.relativeEncoder = sparkMax.getEncoder();
    }

    @Override
    public double getPosition() {
        sparkMax.throwIfClosed();
        final double relativeReportedAbsolutePosition = MathUtil.inputModulus(
                relativeEncoder.getPosition() / relativeEncoder.getPositionConversionFactor(),
                0,
                1
        );
        Logger.getInstance().recordOutput(String.format("SparkMax_%d_relativeEncoderPosition", sparkMax.getDeviceId()), relativeEncoder.getPosition());
        Logger.getInstance().recordOutput(String.format("SparkMax_%d_relativePositionConvFactor", sparkMax.getDeviceId()), relativeEncoder.getPositionConversionFactor());
        Logger.getInstance().recordOutput(String.format("SparkMax_%d_relativeReportedAbsPosition", sparkMax.getDeviceId()), relativeReportedAbsolutePosition);

        return MathUtil.inputModulus(
                ((relativeReportedAbsolutePosition - zeroOffset) * getPositionConversionFactor())
                        + setRawPositionOffset + addedPositionOffset,
                0,
                1
        );
    }

    @Override
    public double getVelocity() {
        sparkMax.throwIfClosed();
        // TODO: magic number for RPM to rots/sec
        return ((relativeEncoder.getVelocity() / relativeEncoder.getVelocityConversionFactor() / 60)
                * getVelocityConversionFactor())
                + setVelocityOffset;
    }

    @Override
    public REVLibError setPositionConversionFactor(final double positionConversionFactor) {
        sparkMax.throwIfClosed();
        this.positionConversionFactor = positionConversionFactor;

        return REVLibError.kOk;
    }

    @Override
    public double getPositionConversionFactor() {
        sparkMax.throwIfClosed();
        return positionConversionFactor;
    }

    @Override
    public REVLibError setVelocityConversionFactor(final double velocityConversionFactor) {
        sparkMax.throwIfClosed();
        this.velocityConversionFactor = velocityConversionFactor;

        return REVLibError.kOk;
    }

    @Override
    public double getVelocityConversionFactor() {
        sparkMax.throwIfClosed();
        return velocityConversionFactor;
    }

    @Override
    public REVLibError setInverted(final boolean inverted) {
        sparkMax.throwIfClosed();
        this.inverted = inverted;

        return REVLibError.kOk;
    }

    @Override
    public boolean getInverted() {
        sparkMax.throwIfClosed();
        return inverted;
    }

    /**
     * Formerly set the average sampling depth of the {@link AbsoluteEncoder},
     * this has been patched and will <b>no longer work</b>.
     * @param depth The average sampling depth of 1, 2, 4, 8, 16, 32, 64, or 128
     * @return {@link REVLibError#kNotImplemented}
     */
    @Override
    public REVLibError setAverageDepth(final int depth) {
        sparkMax.throwIfClosed();
        return REVLibError.kNotImplemented;
    }

    /**
     * Formerly gets the average sampling depth of the {@link AbsoluteEncoder},
     * this has been patched and will <b>no longer work</b>.
     * @return will never return
     * @throws IllegalStateException calling this method will always throw
     */
    @Override
    public int getAverageDepth() {
        sparkMax.throwIfClosed();
        throw new IllegalStateException("NotImplemented!");
    }

    @Override
    public REVLibError setZeroOffset(final double zeroOffset) {
        sparkMax.throwIfClosed();
        this.zeroOffset = zeroOffset / getPositionConversionFactor();

        return REVLibError.kOk;
    }

    @Override
    public double getZeroOffset() {
        sparkMax.throwIfClosed();
        return (getInverted() ? 1 - zeroOffset : zeroOffset) * getPositionConversionFactor();
    }

    /**
     * Sets the supply voltage of the {@link SimSparkMaxAbsoluteEncoder}.
     * This method currently does nothing.
     * @param volts the supply voltage in Volts
     */
    @Override
    public void setSupplyVoltage(final double volts) {}

    @Override
    public void setRawPosition(final double rotations) {
        final double wrappedRotations = MathUtil.inputModulus(rotations, 0, 1);
        this.setRawPositionOffset = wrappedRotations - getPosition();
    }

    @Override
    public void addPosition(double deltaRotations) {
        this.addedPositionOffset += deltaRotations;
    }

    @Override
    public void setVelocity(double rotationsPerSec) {
        this.setVelocityOffset = rotationsPerSec - getVelocity();
    }
}
