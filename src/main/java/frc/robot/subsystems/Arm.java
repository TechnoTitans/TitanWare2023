package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.wrappers.sensors.LimitSwitch;


public class Arm extends SubsystemBase {
    private CANSparkMax extender;
    private CANSparkMax hinge;
    private LimitSwitch limit;
    private double distance;
    private Encoder encoder = new Encoder(0,1);

    public Arm(CANSparkMax extender, CANSparkMax hinge, LimitSwitch limit) {
        this.extender = extender;
        this.hinge = hinge;
        this.limit = limit;
        this.distance = 0;

    }
    public void extend(){
        extender.set(.1);
    }
    public void armPause(){
        extender.set(0);
    }



    public void retract() {
        extender.set(-.1);
    }
    public void tiltHingeDown(CANSparkMax hinge){
        hinge.set(-.1);
    }
    public void tiltHingeUp(CANSparkMax hinge){
        hinge.set(.1);
    }





}
