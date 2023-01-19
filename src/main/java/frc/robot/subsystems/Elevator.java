package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TitanSRX motor;

    private Encoder encoder;
    public void setEncoder(Encoder encoder){
        this.encoder = encoder;
    }
    public Elevator(TitanSRX motor){
        this.motor = motor;
        this.encoder = new Encoder(0, 1);
        encoder.reset();
        encoder.setDistancePerPulse(1./256.);
    }

//we are just using 5 and 50 as a placeholder since we don't know the actual min and max
    /**
     * Sets the direction and speed of the elevator
     * and makes sure that the elevator does not reach
     * maxiumum and the minimum so that the elevator doesn't brake
     *
     * @param speed - Speed from 0 to 1 (or negative for backwards)
     */
    public void set(double speed){
        motor.set(speed);
        if (speed < 0 && this.getDistance() > 5){
            motor.set(speed);
        }
        else if (speed > 0 && this.getDistance() > 50){
            motor.set(speed);
        }
    }
    public double getDistance(){
        return encoder.getDistance() * 28.65;
        //^ 360 / (2 * Math.PI * radius of wheel)
    }
    public void getMotor(){
        return motor;
    }
}