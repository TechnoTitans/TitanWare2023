package frc.robot.subsystems.elevator;

import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.utils.Enums;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public String desiredState = Enums.ElevatorState.ELEVATOR_RESET.toString();
        public String verticalElevatorMode = Enums.ElevatorMode.MOTION_MAGIC.toString();
        public double VEPositionRotations = 0.0;
        public boolean horizontalPositionalControl = false;
        public double HEPositionRotations = 0.0;

        public double verticalElevatorEncoderPosition = 0.0;
        public double verticalElevatorEncoderVelocity = 0.0;
        public double horizontalElevatorEncoderPosition = 0.0;
        public double horizontalElevatorEncoderVelocity = 0.0;

        public boolean verticalElevatorLimitSwitch = false;
        public boolean horizontalElevatorLimitSwitch = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ElevatorIOInputs
     * @see AutoLog
     */
    void updateInputs(final ElevatorIOInputs inputs);

    void config();

    void setDesiredState(final Enums.ElevatorState state);

    Enums.ElevatorState getDesiredState();

    default boolean verticalIsExtended() {
        final Enums.ElevatorState desiredState = getDesiredState();
        return desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_HIGH
                || desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_MID
                || desiredState == Enums.ElevatorState.ELEVATOR_EXTENDED_PLATFORM;
    }
}
