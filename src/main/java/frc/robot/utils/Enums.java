package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Enums {
    public enum VerticalElevatorMode {
        POSITION,
        MOTION_MAGIC,
        DUTY_CYCLE
    }

    public enum HorizontalElevatorMode {
        POSITION,
        DUTY_CYCLE
    }

    public enum ElevatorClawStateType {
        STANDBY,
        INTAKING,
        SCORING
    }

    public enum ElevatorState {
        //DutyCycle back to limit switch to reset encoder
        ELEVATOR_RESET(
                -0.25,
                VerticalElevatorMode.MOTION_MAGIC,
                -0.25,
                HorizontalElevatorMode.DUTY_CYCLE,
                ElevatorClawStateType.STANDBY
        ),
        //Elevator High and Horizontal extended
        ELEVATOR_EXTENDED_HIGH(
                4.9,
                VerticalElevatorMode.MOTION_MAGIC,
                3,
                HorizontalElevatorMode.POSITION,
                ElevatorClawStateType.SCORING
        ),
        //Elevator Mid and Horizontal extended
        ELEVATOR_EXTENDED_MID(
                3.2,
                VerticalElevatorMode.MOTION_MAGIC,
                0.9,
                HorizontalElevatorMode.POSITION,
                ElevatorClawStateType.SCORING
        ),
        //Elevator Platform and Horizontal extended
        ELEVATOR_DOUBLE_SUB(
                4.3,
                VerticalElevatorMode.MOTION_MAGIC,
                0,
                HorizontalElevatorMode.POSITION,
                ElevatorClawStateType.INTAKING
        ),
        //Elevator at normal height
        ELEVATOR_STANDBY(
                0, // -0.25,
                VerticalElevatorMode.MOTION_MAGIC,
                0,
                HorizontalElevatorMode.POSITION,
                ElevatorClawStateType.STANDBY
        ),
        ELEVATOR_CUBE(
                1.3,
                VerticalElevatorMode.MOTION_MAGIC,
                -0.3,
                HorizontalElevatorMode.DUTY_CYCLE,
                ElevatorClawStateType.INTAKING
        ),
        ELEVATOR_SINGLE_SUB(
                2.1,
                VerticalElevatorMode.POSITION,
                0,
                HorizontalElevatorMode.POSITION,
                ElevatorClawStateType.INTAKING
        ),
        ELEVATOR_TIPPED_CONE(
                1.55,
                VerticalElevatorMode.MOTION_MAGIC,
                0.2,
                HorizontalElevatorMode.POSITION,
                ElevatorClawStateType.INTAKING
        );

        final VerticalElevatorMode verticalElevatorMode;
        final double VEControlInput;
        final HorizontalElevatorMode horizontalElevatorMode;
        final double HEControlInput;
        final ElevatorClawStateType elevatorClawStateType;

        public VerticalElevatorMode getVerticalElevatorMode() {
            return verticalElevatorMode;
        }

        public double getVEControlInput() {
            return VEControlInput;
        }

        public HorizontalElevatorMode getHorizontalElevatorMode() {
            return horizontalElevatorMode;
        }

        public double getHEControlInput() {
            return HEControlInput;
        }

        public ElevatorClawStateType getElevatorStateType() {
            return elevatorClawStateType;
        }

        ElevatorState(
                final double VEControlInput,
                final VerticalElevatorMode verticalElevatorMode,
                final double HEControlInput,
                final HorizontalElevatorMode horizontalElevatorMode,
                final ElevatorClawStateType elevatorClawStateType
        ) {
            this.VEControlInput = VEControlInput;
            this.verticalElevatorMode = verticalElevatorMode;
            this.HEControlInput = HEControlInput;
            this.horizontalElevatorMode = horizontalElevatorMode;
            this.elevatorClawStateType = elevatorClawStateType;
        }
    }

    public enum ClawTiltControlMode {
        POSITION,
        DUTY_CYCLE
    }

    public enum ClawOpenCloseControlMode {
        POSITION(ControlMode.Position),
        DUTY_CYCLE(ControlMode.PercentOutput);

        private final ControlMode controlMode;
        ClawOpenCloseControlMode(final ControlMode controlMode) {
            this.controlMode = controlMode;
        }

        public ControlMode getControlMode() {
            return controlMode;
        }
    }

    public enum ClawState {
        //Claw shoot cube
        CLAW_OUTTAKE(
                -0.1,
                ClawTiltControlMode.POSITION,
                0.295,
                ClawOpenCloseControlMode.DUTY_CYCLE,
                0.2,
                ElevatorClawStateType.SCORING
        ),
        //Claw shoot cube
        CLAW_OUTTAKE_HYBRID(
                -0.2,
                ClawTiltControlMode.POSITION,
                .295,
                ClawOpenCloseControlMode.DUTY_CYCLE,
                .3,
                ElevatorClawStateType.SCORING
        ),
        //Claw tilted down and closed
        CLAW_HOLDING(
                0.2,
                ClawTiltControlMode.POSITION,
                0,
                ClawOpenCloseControlMode.DUTY_CYCLE,
                -0.37,
                ElevatorClawStateType.STANDBY
        ),
        //Claw tilted down and open cone
        CLAW_INTAKING_CONE(
                0.5,
                ClawTiltControlMode.POSITION,
                0.3,
                ClawOpenCloseControlMode.POSITION,
                0.024,
                ElevatorClawStateType.INTAKING
        ),
        //Claw tilted down and open cube
        CLAW_INTAKING_CUBE(
                0.5,
                ClawTiltControlMode.POSITION,
                0.3,
                ClawOpenCloseControlMode.POSITION,
                0.171,
                ElevatorClawStateType.INTAKING
        ),
        //Claw tilted down and standby wheel speed
        CLAW_STANDBY(
                0.2,
                ClawTiltControlMode.POSITION,
                0,
                ClawOpenCloseControlMode.POSITION,
                0,
                ElevatorClawStateType.STANDBY
        ),
        //Drop claw to outtake height
        CLAW_DROP(
                0.3,
                ClawTiltControlMode.POSITION,
                0.2,
                ClawOpenCloseControlMode.DUTY_CYCLE,
                -0.37,
                ElevatorClawStateType.SCORING
        ),
        CLAW_ANGLE_SHOOT(
                0.2,
                ClawTiltControlMode.POSITION,
                0.12,
                ClawOpenCloseControlMode.DUTY_CYCLE,
                -0.37,
                ElevatorClawStateType.SCORING
        ),
        CLAW_SHOOT_HIGH(
                -0.8,
                ClawTiltControlMode.POSITION,
                0.12,
                ClawOpenCloseControlMode.DUTY_CYCLE,
                -0.37,
                ElevatorClawStateType.SCORING
        ),
        CLAW_SHOOT_LOW(
                -0.3,
                ClawTiltControlMode.POSITION,
                0.12,
                ClawOpenCloseControlMode.DUTY_CYCLE,
                -0.37,
                ElevatorClawStateType.SCORING
        ),
        CLAW_ANGLE_CUBE(
                0.6,
                ClawTiltControlMode.POSITION,
                0.4,
                ClawOpenCloseControlMode.POSITION,
                0.171,
                ElevatorClawStateType.INTAKING
        ),
        SINGLE_SUB(
                0.5,
                ClawTiltControlMode.POSITION,
                0.2,
                ClawOpenCloseControlMode.POSITION,
                0.049,
                ElevatorClawStateType.INTAKING
        ),
        TIPPED_CONE(
                0.5,
                ClawTiltControlMode.POSITION,
                0.45,
                ClawOpenCloseControlMode.POSITION,
                0.024,
                ElevatorClawStateType.INTAKING
        );

        final double intakeWheelsPercentOutput;
        final ClawTiltControlMode clawTiltControlMode;
        final double tiltControlInput;
        final ClawOpenCloseControlMode clawOpenCloseControlMode;
        final double openCloseControlInput;
        final ElevatorClawStateType elevatorClawStateType;

        public double getIntakeWheelsPercentOutput() {
            return intakeWheelsPercentOutput;
        }

        public ClawTiltControlMode getClawTiltControlMode() {
            return clawTiltControlMode;
        }

        public double getTiltControlInput() {
            return tiltControlInput;
        }

        public ClawOpenCloseControlMode getClawOpenCloseControlMode() {
            return clawOpenCloseControlMode;
        }

        public double getOpenCloseControlInput() {
            return openCloseControlInput;
        }

        public ElevatorClawStateType getElevatorStateType() {
            return elevatorClawStateType;
        }

        ClawState(final double intakeWheelsPercentOutput,
                  final ClawTiltControlMode clawTiltControlMode,
                  final double tiltControlInput,
                  final ClawOpenCloseControlMode clawOpenCloseControlMode,
                  final double openCloseControlInput,
                  final ElevatorClawStateType elevatorClawStateType
        ) {
            this.intakeWheelsPercentOutput = intakeWheelsPercentOutput;
            this.clawTiltControlMode = clawTiltControlMode;
            this.tiltControlInput = tiltControlInput;
            this.clawOpenCloseControlMode = clawOpenCloseControlMode;
            this.openCloseControlInput = openCloseControlInput;
            this.elevatorClawStateType = elevatorClawStateType;
        }
    }

    public enum CANdleState {
        OFF(),
        YELLOW(200, 100, 0),
        PURPLE(200, 0, 150);

        final Color color;
        CANdleState(final Color color) {
            this.color = color;
        }

        CANdleState(final int r, final int g, final int b) {
            this(new Color(r, g, b));
        }

        CANdleState() {
            this(Color.kBlack);
        }

        public Color getColor() {
            return color;
        }

        /**
         * Get the Red component of the color RGB
         * @return the Red component [0, 255]
         */
        public int getR() {
            return (int)(getColor().red * 255);
        }

        /**
         * Get the Green component of the color RGB
         * @return the Green component [0, 255]
         */
        public int getG() {
            return (int)(getColor().green * 255);
        }

        /**
         * Get the Blue component of the color RGB
         * @return the Blue component [0, 255]
         */
        public int getB() {
            return (int)(getColor().blue * 255);
        }
    }

    public enum DriverProfile {
        DRIVER1(1, 1),
        DRIVER2(1.1, 1.1),
        DEFAULT(1, 1);

        final double throttleSensitivity;
        final double rotationalSensitivity;

        DriverProfile(final double throttleSensitivity, final double rotationalSensitivity) {
            this.throttleSensitivity = throttleSensitivity;
            this.rotationalSensitivity = rotationalSensitivity;
        }

        public double getThrottleSensitivity() {
            return throttleSensitivity;
        }

        public double getRotationalSensitivity() {
            return rotationalSensitivity;
        }
    }

    public enum SwerveSpeed {
        FAST(Units.feetToMeters(13), 0.5),
        NORMAL(Units.feetToMeters(6), 0.35),
        SLOW(Units.feetToMeters(2), 0.25);

        final double throttleWeight;
        final double rotateWeight;

        SwerveSpeed(final double throttleWeight, final double rotateWeight) {
            this.throttleWeight = throttleWeight / Constants.Swerve.TELEOP_MAX_SPEED;
            this.rotateWeight = (Math.PI * rotateWeight) / Constants.Swerve.TELEOP_MAX_ANGULAR_SPEED;
        }

        public double getThrottleWeight() {
            return throttleWeight;
        }

        public double getRotateWeight() {
            return rotateWeight;
        }
    }
}
