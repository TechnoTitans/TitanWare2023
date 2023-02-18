package frc.robot.commands.subsystems;

import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;
import frc.robot.wrappers.motors.TitanMAX;

public class ElevatorControl extends CommandBase {
    private final Elevator elevator;
    private final TalonFX verticalElevatorMotor;
    private final TitanMAX horizontalElevatorMotor;
    private final MotionMagicDutyCycle motionMagicDutyCycle;

    private Enums.ElevatorState currentState;

    private double
            HETargetRotations = 0, //Horizontal Elevator Target Ticks
            VEPosition = 0; //Vertical Elevator Target Ticks

    public ElevatorControl(Elevator elevator) {
        this.elevator = elevator;
        this.verticalElevatorMotor = elevator.getVerticalElevatorMotor();
        this.horizontalElevatorMotor = elevator.getHorizontalElevatorMotor();
        this.motionMagicDutyCycle = new MotionMagicDutyCycle(0, true, 0, 0, false);

        addRequirements(elevator);
    }

    private void setState(Enums.ElevatorState state) {
        //TODO TUNE THIS
        switch (state) {
            case ELEVATOR_EXTENDED_HIGH:
                VEPosition = 50;
                HETargetRotations = 30;
                break;
            case ELEVATOR_EXTENDED_MID:
                VEPosition = 40;
                HETargetRotations = 20;
                break;
            case ELEVATOR_EXTENDED_GROUND:
                VEPosition = 17;
                HETargetRotations = 15;
                break;
            case ELEVATOR_STANDBY:
                VEPosition = 0;
                HETargetRotations = 10;
                break;
            case ELEVATOR_EXTENDED_PLATFORM:
                VEPosition = 50;
                HETargetRotations = 15;
                break;
            case ELEVATOR_PREGAME:
                HETargetRotations = 0;
                VEPosition = 0;
                break;
            default:
                break;
        }
    }

    public boolean isAtWantedState() {
        return MathMethods.withinRange(
                verticalElevatorMotor.getPosition().getValue(),
                VEPosition,
                0.1) &&
                MathMethods.withinRange(
                        horizontalElevatorMotor.getAlternateEncoder(8192).getPosition(),
                        HETargetRotations,
                        0.1);
    }

    @Override
    public void execute() {
        Enums.ElevatorState newState = elevator.getCurrentState();
        if (newState != currentState) {
            currentState = newState;
            setState(currentState);
        }

        verticalElevatorMotor.setControl(
                motionMagicDutyCycle.withPosition(VEPosition));

        horizontalElevatorMotor.set(
                CANSparkMax.ControlType.kPosition,
                HETargetRotations);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}