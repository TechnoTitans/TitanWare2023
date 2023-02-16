package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Enums;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;

public class ClawControl extends CommandBase {
    private final Claw claw;
    private final TitanSRX clawWheelMotor, clawOpenCloseMotor;
    private final TitanMAX clawTiltNeo;

    private ControlMode openCloseControlMode;
    private double
            speed = 0, //Claw Intake Wheel Speed
            tiltRotations = 0, //Claw Tilt Rotations
            openCloseControl = 0; //Claw Open Close Ticks

    public ClawControl(Claw claw) {
        this.claw = claw;
        this.clawWheelMotor = claw.getClawWheelMotor();
        this.clawOpenCloseMotor = claw.getClawOpenCloseMotor();
        this.clawTiltNeo = claw.getClawTiltNeo();
        addRequirements(claw);
    }

    private void setState(Enums.ClawState state) {
        switch (state) {
            case CLAW_HOLDING:
                speed = 0.15;
                tiltRotations = 0;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.2;
                break;
            case CLAW_OUTTAKE:
                speed = -0.1;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = 0;
                break;
            case CLAW_INTAKING:
                speed = 0.3;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.MotionMagic;
                openCloseControl = -0.1;
                break;
            case CLAW_DROP_CONE:
                speed = 0.2;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = 0.2;
                break;
            case CLAW_STANDBY:
                speed = 0.2;
                tiltRotations = 500;
                openCloseControlMode = ControlMode.MotionMagic;
                openCloseControl = 0.2;
                break;
            default:
                break;
        }
    }

    @Override
    public void execute() {
        setState(claw.getCurrentState());

        clawWheelMotor.set(
                ControlMode.PercentOutput,
                speed);

        clawOpenCloseMotor.set(
                openCloseControlMode,
                openCloseControl);

        clawTiltNeo.set(
                CANSparkMax.ControlType.kSmartMotion,
                tiltRotations);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
