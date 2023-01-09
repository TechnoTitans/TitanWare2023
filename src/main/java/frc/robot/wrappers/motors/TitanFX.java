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

	public double P, I, D;
	private double error;
	private double previousError;
	private double target;
	private double integral;

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

	public TitanFX(int channel, String canBus, boolean reversed) {
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

	public void setFiltered(double speed) {
		speed = MathUtil.clamp(speed, -1, 1);
		super.set(ControlMode.PercentOutput, filter.calculate(speed));
	}

	public void setVelocityPID(double rpm) {
		target = rpm;
		double ticksper100ms = rpm * 2048 / 600;
		super.set(ControlMode.Velocity, ticksper100ms);
	}

	public void setVelocityPIDFiltered(double rpm) {
		rpm = rpm * 2048 / (60 * 10);
		super.set(ControlMode.Velocity, PIDCalculate(filter.calculate(rpm)));
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


	public double getSelectedSensorPosition() {
		return super.getSelectedSensorPosition();
	}

	public double getSelectedSensorVelocity() {
		return super.getSelectedSensorVelocity();
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

	public void configPID(double P, double I, double D) {
		this.P = P;
		this.I = I;
		this.D = D;
	}

	public double PIDCalculate(double target) {
		this.target = target;
		error = this.target - (getSpeed() / 5);
		integral += (error * .02);
		double derivative = (error - previousError) / .02;
//		return P * error + I * integral + D * derivative;
		return target;
	}

	public double getError() {
		return error;
	}

	public double getTarget() {
		return target;
	}

	public void setVoltage(double outputVolts) {
		super.setVoltage(outputVolts);
	}
}
