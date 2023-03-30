package frc.robot.wrappers.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/*
 * Motor control (talonFX)
 * TitanFX is our enhanced version of the regular TalonFX code
 */
@SuppressWarnings("unused")
public class TitanFX extends WPI_TalonFX implements MotorController {

    private static final int TIMEOUT_MS = 30;

    private TitanFX brownoutFollower = null;
    private boolean brownout = false;

    private SlewRateLimiter filter;

    /**
     * Constructor for a TalonFX motor
     *
     * @param channel  The port where the TalonFX is plugged in.
     * @param reversed If the TalonFX should invert the signal.
     */
    public TitanFX(int channel, boolean reversed) {
        super(channel);
        super.setInverted(reversed);
    }

    public TitanFX(int channel, boolean reversed, String canBus) {
        super(channel, canBus);
        super.setInverted(reversed);
    }

    public TitanFX(int channel, boolean reversed, SlewRateLimiter filter) {
        super(channel);
        super.setInverted(reversed);
        this.filter = filter;
    }

    /**
     * Set the speed of the TalonFX.
     *
     * @param speed -- Speed from 0 to 1 (or negative for backwards)
     */

    public void set(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        super.set(ControlMode.PercentOutput, speed);
    }

    public double getClosedLoopError(int slotIdx) {
        return super.getClosedLoopError(slotIdx);
    }

    public void brake() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Coast);
    }

    public void limitCurrent(int amps) {
        super.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amps, amps, 1));
    }

    public void resetEncoder() {
        super.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public int getChannel() {
        return super.getDeviceID();
    }


    public boolean isReversed() {
        return super.getInverted();
    }


    public double getPercentSpeed() {
        return super.getMotorOutputPercent();
    }

    public double getCurrent() {
        return super.getStatorCurrent();
    }

    public double getSpeed() {
        return super.getSelectedSensorVelocity();
    }

    @Override
    public void stopMotor() {
        set(0);
    }

    public void follow(TitanFX other) {
        other.brownoutFollower = this;
        this.set(ControlMode.Follower, other.getChannel());
    }

    public void enableBrownoutProtection() {
        if (brownoutFollower != null) {
            brownoutFollower.coast();
        }
        brownout = true;
    }

    public void disableBrownoutProtection() {
        if (brownoutFollower != null && brownout) {
            brownoutFollower.setNeutralMode(NeutralMode.Brake);
            brownoutFollower.set(ControlMode.Follower, getChannel());
        }
        brownout = false;
    }

    public void setVoltage(double outputVolts) {
        super.setVoltage(outputVolts);
    }

}
