package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.motors.TitanMAX;
import frc.robot.wrappers.motors.TitanSRX;

public class ClawControl extends CommandBase {
    private final Claw claw;
    private final TitanSRX clawWheelMotor, clawOpenCloseMotor;
    private final TitanMAX clawTiltNeo;

    private Enums.ClawState currentState;
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
                speed = 0.2;
                tiltRotations = 0;
                openCloseControl = 15;
                break;
            case CLAW_STANDBY:
                speed = 0.2;
                tiltRotations = .02;
                openCloseControl = 50;
                break;
            case CLAW_OUTTAKE:
                speed = -0.2;
                tiltRotations = .3;
                openCloseControl = 65;
                break;
            case CLAW_INTAKE_CONE:
                speed = 0.3;
                tiltRotations = .3;
                openCloseControl = 15;
                break;
            case CLAW_INTAKE_CUBE:
                speed = 0.3;
                tiltRotations = .3;
                openCloseControl = 30;
                break;
            default:
                break;
        }
    }

    public boolean isAtWantedState() {
        return MathMethods.withinRange(
                clawOpenCloseMotor.getSelectedSensorPosition(),
                openCloseControl,
                0.1) &&
                MathMethods.withinRange(
                        clawTiltNeo.getABSRevBoreThroughEncoder().getPosition(),
                        tiltRotations,
                        0.1);
    }

    @Override
    public void execute() {
        Enums.ClawState newState = claw.getTargetState();
        if (newState != currentState) {
            currentState = newState;
            setState(currentState);
        }
        SmartDashboard.putString("Claw State", newState.toString());

        clawWheelMotor.set(
                ControlMode.PercentOutput,
                speed);

        double error = openCloseControl - claw.getOpenCloseEncPosition();
        clawOpenCloseMotor.set(
                ControlMode.PercentOutput,
                error*.01);

        clawTiltNeo.set(
                CANSparkMax.ControlType.kPosition,
                tiltRotations);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
