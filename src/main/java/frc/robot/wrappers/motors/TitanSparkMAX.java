package frc.robot.wrappers.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.math.MathUtil;

public class TitanSparkMAX extends CANSparkMax {
    public TitanSparkMAX(
            final int deviceId,
            final MotorType type
    ) {
        super(deviceId, type);
    }

    @Override
    public void set(final double speed) {
        super.set(MathUtil.clamp(speed, -1, 1));
    }

    public void set(final ControlType controlType, final double value) {
        this.getPIDController().setReference(value, controlType);
    }

    /**
     * Set the free speed of the motor being simulated.
     *
     * @param freeSpeed the free speed (RPM) of the motor connected to spark max
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSimFreeSpeed(final double freeSpeed) {
        throwIfClosed();
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetSimFreeSpeed(sparkMaxHandle, (float)freeSpeed));
    }

    /**
     * Set the stall torque of the motor being simulated.
     *
     * @param stallTorque The stall torque (N m) of the motor connected to sparkmax
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSimStallTorque(final double stallTorque) {
        throwIfClosed();
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetSimStallTorque(sparkMaxHandle, (float) stallTorque));
    }

    /**
     * Get if the internal SparkMAX has been closed.
     * @return true if already closed, false if not.
     * @see com.revrobotics.CANSparkMaxLowLevel#isClosed
     */
    public boolean isClosed() {
        return isClosed.get();
    }
}
