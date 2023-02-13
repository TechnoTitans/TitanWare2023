package frc.robot.wrappers.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/*
 * Motor control (talonSRX)
 * TitanSRX is our enhanced version of the regular TalonSRX code
 */
@SuppressWarnings("unused")
public class TitanSRX extends WPI_TalonSRX implements MotorController {

    private TitanSRX brownoutFollower = null;
    private Encoder encoder;
    private boolean brownout = false;

    /**
     * Constructor for a TalonSRX motor
     *
     * @param channel  The port where the TalonSRX is plugged in.
     * @param reversed If the TalonSRX should invert the signal.
     */
    public TitanSRX(int channel, boolean reversed) {
        super(channel);
        super.setInverted(reversed);
    }

    public TitanSRX(int channel, boolean reversed, Encoder encoder) {
        super(channel);
        super.setInverted(reversed);
        this.encoder = encoder;
    }

    /**
     * Set the speed of the TalonSRX.
     *
     * @param speed -- Speed from 0 to 1 (or negative for backwards)
     */

    public void set(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        super.set(ControlMode.PercentOutput, speed);
    }

    public void setEncoder(Encoder encoder) {
        this.encoder = encoder;
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

    public boolean hasEncoder() {
        return getEncoder() != null;
    }


    public Encoder getEncoder() {
        return encoder;
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


    public void stop() {
        set(0);
    }

    public void follow(TitanSRX other) {
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
