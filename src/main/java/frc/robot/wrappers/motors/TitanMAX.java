package frc.robot.wrappers.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;

public class TitanMAX extends CANSparkMax  {
    public TitanMAX(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public TitanMAX(int deviceId, MotorType type, boolean inverted) {
        super(deviceId, type);
        this.setInverted(inverted);
    }

    public void set(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        super.set(speed);
    }

    public void set(ControlType controlType, double value) {
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

    public void follow(TitanMAX other) {
        super.follow(other);
    }

    public SparkMaxPIDController getPIDController() {
        return super.getPIDController();
    }

    public RelativeEncoder getIntegratedEncoder() {
        return super.getEncoder();
    }

    public RelativeEncoder getAlternateEncoder(int countsPerRev) {
        return super.getAlternateEncoder(countsPerRev);
    }
}
