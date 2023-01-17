package frc.robot.commands.teleop;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.wrappers.sensors.LimitSwitch;
import frc.robot.utils.MathMethods;








public class ArmExtend extends CommandBase {
    private final Arm arm;
    private final XboxController controller;
    private Encoder encoder = new Encoder(0,1);
    private CANSparkMax extender;
    public ArmExtend(Arm arm, XboxController controller, CANSparkMax extender){
        this.arm = arm;
        this.controller = controller;
        this.extender = extender;
        addRequirements(arm);
    }
    @Override
    public void execute(){
        if(controller.getLeftBumper()){
            encoder.reset();
            arm.extend();
            if (encoder.getDistance() == 39.75*Math.PI/90){
                arm.armPause();
            }

        }

        if(controller.getRightBumper()){
            encoder.reset();
            arm.extend();
            if(encoder.getDistance() == 23.5*Math.PI/90){
                arm.armPause();
            }
        if(controller.getAButton()){
            if(encoder.getDistance() == 39.75*Math.PI/90){

            }
        }

        }
    }
    @Override
    public boolean isFinished() {
        return false;
    };

}
