package frc.robot.wrappers.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;

@SuppressWarnings("unused")
public class TitanMAX extends CANSparkMax {

    public TitanMAX(
            final int deviceId,
            final MotorType type
    ) {
        super(deviceId, type);
    }

    public TitanMAX(
            final int deviceId,
            final MotorType type,
            final boolean inverted
    ) {
        super(deviceId, type);
        super.setInverted(inverted);
    }

    public void set(final double speed) {
        final double clampedSpeed = MathUtil.clamp(speed, -1, 1);
        super.set(clampedSpeed);
    }

    public void set(
            final ControlType controlType,
            final double value
    ) {
        this.getPIDController().setReference(value, controlType);
    }

    public void stop() {
        this.set(0);
    }

    public void brake() {
        super.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        super.setIdleMode(IdleMode.kCoast);
    }

    public void follow(final TitanMAX other) {
        super.follow(other);
    }

    public void currentLimit(final int stallLimit) {
        super.setSmartCurrentLimit(stallLimit);
    }

    public void currentLimit(
            final int stallLimit,
            final int freeLimit
    ) {
        super.setSmartCurrentLimit(stallLimit, freeLimit);
    }

    public void currentLimit(
            final int stallLimit,
            final int freeLimit,
            final int limitRPM
    ) {
        super.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    }

    public SparkMaxPIDController getPIDController() {
        return super.getPIDController();
    }

    public RelativeEncoder getIntegratedEncoder() {
        return super.getEncoder();
    }
}
