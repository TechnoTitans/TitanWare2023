package frc.robot.wrappers.sensors;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

@SuppressWarnings("unused")
public class QuadEncoder implements Encoder {

    private final BaseMotorController talonSRX;
    public double PULSES_PER_ROTATION = 2048;
    private final double inchesPerPulse; // configure

    public QuadEncoder(BaseMotorController talonSRX, double inchesPerPulse, boolean reversed) {
        this.talonSRX = talonSRX;
        this.inchesPerPulse = inchesPerPulse;
        // this.talonSRX.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        this.talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        talonSRX.setSensorPhase(reversed);
    }

    public QuadEncoder(BaseMotorController talonSRX, double inchesPerPulse, boolean reversed, double PULSES_PER_ROTATION) {
        this.talonSRX = talonSRX;
        this.inchesPerPulse = inchesPerPulse;
        // this.talonSRX.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        this.talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        talonSRX.setSensorPhase(reversed);
        this.PULSES_PER_ROTATION = PULSES_PER_ROTATION;
    }

    /**
     * The total distance that the motor has traveled (in inches)
     *
     * @return the total distance in inches
     */
    @Override
    public double getDistance() {
        return talonSRX.getSelectedSensorPosition(0) * inchesPerPulse;
    }

    /**
     * Gets speed of the TalonSRX in RPM
     */
    // speed = enc counts / 100 ms
    // (speed * 60 secs)
    // --------------------------------------
    // 2048 encoder counts * 100 milliseconds
    @Override
    public double getSpeed() {
        return (talonSRX.getSelectedSensorVelocity(0) * 60) / (PULSES_PER_ROTATION * 0.1);
    }

    @Override
    public double getSpeedInches() {
        return talonSRX.getSelectedSensorVelocity(0) * 10 * inchesPerPulse;
    }

    @Override
    public void reset() {
        talonSRX.setSelectedSensorPosition(0, 0, 0);
    }

    @Override
    public void resetToRaw(int position) {
        talonSRX.setSelectedSensorPosition(position, 0, 0);
    }

    @Override
    public void resetTo(double position) {
        talonSRX.setSelectedSensorPosition((int) (position / inchesPerPulse), 0, 0);
    }

    public double getRawPosition() {
        return talonSRX.getSelectedSensorPosition(0);
    }

    @Override
    public double getInchesPerPulse() {
        return inchesPerPulse;
    }

    public BaseMotorController getTalon() {
        return talonSRX;
    }
}
