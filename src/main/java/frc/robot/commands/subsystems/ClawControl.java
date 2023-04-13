package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final CANCoder clawTiltEncoder;

    private Enums.ClawState currentState;
    private ControlMode openCloseControlMode;
    private Enums.ClawMode clawMode;
    private final ProfiledPIDController tiltPID;

    private double
            speed = 0, //Claw Intake Wheel Speed
            tiltRotations = 0, //Claw Tilt Rotations
            openCloseControl = 0; //Claw Open Close Ticks

    public ClawControl(Claw claw) {
        this.claw = claw;
        this.clawWheelMotor = claw.getClawWheelMotor();
        this.clawOpenCloseMotor = claw.getClawOpenCloseMotor();
        this.clawTiltNeo = claw.getClawTiltNeo();
        this.clawTiltEncoder = claw.getClawTiltEncoder();

        this.tiltPID = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(3, 5));

        addRequirements(claw);
    }

    private void setState(Enums.ClawState state) {
        switch (state) {
            case CLAW_HOLDING:
                speed = 0.2;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = 0;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.37; //-0.37
                break;
            case CLAW_STANDBY:
                speed = 0.2;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = 0;
                openCloseControlMode = ControlMode.Position;
                openCloseControl = 260;
                break;
            case CLAW_OUTTAKE:
                speed = -0.1;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .295;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = .2;
                break;
            case CLAW_OUTTAKE_HYBIRD:
                speed = -0.2;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .295;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = .3;
                break;
            case CLAW_INTAKING_CONE:
                speed = 0.5;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .3;
                openCloseControlMode = ControlMode.Position;
                openCloseControl = 100;
                break;
            case CLAW_INTAKING_CUBE:
                speed = 0.5;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .3;
                openCloseControlMode = ControlMode.Position;
                openCloseControl = 700;
                break;
            case CLAW_DROP:
                speed = 0.3;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .2;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.37;
                break;
            case CLAW_ANGLE_SHOOT:
                speed = 0.2;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .12;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.37;
                break;
            case CLAW_SHOOT_HIGH:
                speed = -0.8;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .12;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.37;
                break;
            case CLAW_SHOOT_LOW:
                speed = -0.3;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .12;
                openCloseControlMode = ControlMode.PercentOutput;
                openCloseControl = -0.37;
                break;
            case CLAW_ANGLE_CUBE:
                speed = 0.5;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .4;
                openCloseControlMode = ControlMode.Position;
                openCloseControl = 700;
                break;
            case SINGLE_SUB:
                speed = 0.5;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .2;
                openCloseControlMode = ControlMode.Position;
                openCloseControl = 200;
                break;
            case TIPPED_CONE:
                speed = 0.5;
                clawMode = Enums.ClawMode.POSITION;
                tiltRotations = .45;
                openCloseControlMode = ControlMode.Position;
                openCloseControl = 100;
                break;
            default:
                break;
        }
    }

    public boolean isAtWantedState() {
        return MathMethods.withinRange(
                clawOpenCloseMotor.getSelectedSensorPosition(),
                openCloseControl,
                5) &&
                MathMethods.withinRange(
                        clawTiltEncoder.getPosition(),
                        tiltRotations,
                        5);
    }

    @Override
    public void initialize() {
        setState(Enums.ClawState.CLAW_STANDBY);
    }

    @Override
    public void execute() {
        Enums.ClawState newState = claw.getTargetState();
        if (newState != currentState) {
            currentState = newState;
            setState(currentState);
        }

        if (claw.getClawTiltLimitSwitch().get() && clawMode == Enums.ClawMode.DUTY_CYCLE) {
            clawTiltEncoder.setPosition(0);
            tiltRotations = 0.1;
        }

        clawWheelMotor.set(
                ControlMode.PercentOutput,
                speed);

        clawOpenCloseMotor.set(
                openCloseControlMode,
                openCloseControl);

        switch (clawMode) {
            case POSITION:
                clawTiltNeo.set(tiltPID.calculate(clawTiltEncoder.getAbsolutePosition(), tiltRotations));
                break;
            case DUTY_CYCLE:
                clawTiltNeo.set(tiltRotations);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
