package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Enums;
import frc.robot.utils.MathUtils;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    protected static final String logKey = "Elevator";

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs;

    private final boolean hasSimSolver;
    private final ElevatorSimSolver elevatorSimSolver;

    private Enums.ElevatorState desiredState = Enums.ElevatorState.ELEVATOR_RESET;
    private Enums.ElevatorState currentState = desiredState;

    public Elevator(final ElevatorIO elevatorIO, final ElevatorSimSolver elevatorSimSolver) {
        this.elevatorIO = elevatorIO;
        this.inputs = new ElevatorIOInputsAutoLogged();

        this.hasSimSolver = elevatorSimSolver != null;
        this.elevatorSimSolver = elevatorSimSolver;

        setDesiredState(desiredState);
    }

    public Elevator(final ElevatorIO elevatorIO) {
        this(elevatorIO, null);
    }

    @Override
    public void periodic() {
        elevatorIO.periodic();

        elevatorIO.updateInputs(inputs);
        Logger.getInstance().processInputs(logKey, inputs);

        final boolean atDesiredState = isAtDesiredState();

        Logger.getInstance().recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.getInstance().recordOutput(logKey + "/DesiredState", desiredState.toString());
        Logger.getInstance().recordOutput(logKey + "/AtDesiredState", atDesiredState);

        Logger.getInstance().recordOutput(
                logKey + "/VerticalMode", currentState.getVerticalElevatorMode().toString()
        );

        Logger.getInstance().recordOutput(
                logKey + "/HorizontalMode", currentState.getHorizontalElevatorMode().toString()
        );
    }

    /**
     * See {@link ElevatorIO#setDesiredState(Enums.ElevatorState)}
     */
    public void setDesiredState(final Enums.ElevatorState desiredState) {
        this.desiredState = desiredState;
        elevatorIO.setDesiredState(desiredState);
    }

    public boolean isAtDesiredState() {
        if (currentState == desiredState) {
            return true;
        } else {
            final Enums.VerticalElevatorMode verticalElevatorMode = currentState.getVerticalElevatorMode();
            final Enums.HorizontalElevatorMode horizontalElevatorMode = currentState.getHorizontalElevatorMode();

            final boolean isAtDesired =
                    MathUtils.withinTolerance(
                            switch (verticalElevatorMode) {
                                case POSITION, MOTION_MAGIC -> inputs.verticalElevatorEncoderPosition;
                                case DUTY_CYCLE -> inputs.verticalElevatorMotorDutyCycle;
                            },
                            inputs.VEControlInput,
                            0.05
                    ) && MathUtils.withinTolerance(
                            switch (horizontalElevatorMode) {
                                case POSITION -> inputs.horizontalElevatorEncoderPosition;
                                case DUTY_CYCLE -> inputs.horizontalElevatorMotorDutyCycle;
                            },
                            inputs.HEControlInput,
                            0.05
                    );

            if (isAtDesired) {
                currentState = desiredState;
            }

            return isAtDesired;
        }
    }

    public Enums.ElevatorState getCurrentState() {
        return currentState;
    }

    /**
     * Get the desired {@link frc.robot.utils.Enums.ElevatorState}
     * @return the currently desired {@link frc.robot.utils.Enums.ElevatorState}
     * @see frc.robot.utils.Enums.ElevatorState
     */
    public Enums.ElevatorState getDesiredState() {
        return desiredState;
    }

    /**
     * Get the {@link frc.robot.subsystems.elevator.ElevatorSimSolver.ElevatorSimState} (simulation state) of the
     * elevator.
     * <p>
     *     The implementation spec requires that this throw an {@link UnsupportedOperationException} or
     *     otherwise a {@link RuntimeException} if an {@link ElevatorSimSolver} does not exist.
     * </p>
     * @return the current {@link frc.robot.subsystems.elevator.ElevatorSimSolver.ElevatorSimState}
     */
    public ElevatorSimSolver.ElevatorSimState getElevatorSimState() {
        if (hasSimSolver) {
            return elevatorSimSolver.getElevatorSimState();
        }

        throw new UnsupportedOperationException("Attempted to GetElevatorSimState without a SimSolver!");
    }

    /**
     * Get whether the vertical part of the elevator is extended upwards
     * @return true if extended, false if not
     */
    public boolean verticalIsExtended() {
        final Enums.ElevatorState desiredState = getDesiredState();
        return desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH
                || desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_MID
                || desiredState == Enums.ElevatorState.ELEVATOR_DOUBLE_SUB;
    }
}