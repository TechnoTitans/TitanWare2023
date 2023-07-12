package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Enums {
    public enum ElevatorState {
        //DutyCycle back to limit switch to reset encoder
        ELEVATOR_RESET(
                Enums.ElevatorMode.MOTION_MAGIC,
                -0.25,
                false,
                -0.3
        ),
        //Elevator High and Horizontal extended
        ELEVATOR_EXTENDED_HIGH(
                Enums.ElevatorMode.POSITION,
                5,
                true,
                3
        ),
        ELEVATOR_EXTENDED_MID(
                Enums.ElevatorMode.POSITION,
                3.2,
                true,
                0.9
        ), //Elevator Mid and Horizontal extended
        ELEVATOR_EXTENDED_PLATFORM(
                Enums.ElevatorMode.POSITION,
                4.3,
                true,
                0
        ), //Elevator Platform and Horizontal extended
        ELEVATOR_STANDBY(
                ElevatorMode.MOTION_MAGIC,
                -0.25,
                true,
                0
        ), //Elevator at normal height
        ELEVATOR_CUBE(
                Enums.ElevatorMode.POSITION,
                1.3,
                false,
                -0.3
        ),
        SINGLE_SUB(
                Enums.ElevatorMode.POSITION,
                2.1,
                true,
                0
        ),
        ELEVATOR_TIPPED_CONE(
                Enums.ElevatorMode.POSITION,
                1.55,
                true,
                0.2
        );

        final ElevatorMode verticalElevatorMode;
        final double VEPositionRotations;
        final boolean horizontalPositionalControl;
        final double HEPositionRotation;

        public ElevatorMode getVerticalElevatorMode() {
            return verticalElevatorMode;
        }

        public double getVEPositionRotations() {
            return VEPositionRotations;
        }

        public boolean isHorizontalPositionalControl() {
            return horizontalPositionalControl;
        }

        public double getHEPositionRotation() {
            return HEPositionRotation;
        }

        ElevatorState(
                final ElevatorMode verticalElevatorMode,
                final double VEPositionRotations,
                final boolean horizontalPositionalControl,
                final double HEPositionRotation
        ) {
            this.verticalElevatorMode = verticalElevatorMode;
            this.VEPositionRotations = VEPositionRotations;
            this.horizontalPositionalControl = horizontalPositionalControl;
            this.HEPositionRotation = HEPositionRotation;

        }
    }

    public enum ElevatorMode {
        POSITION,
        MOTION_MAGIC,
        DUTY_CYCLE;
    }

    public enum ClawControlMode {
        POSITION,
        DUTY_CYCLE
    }

    public enum ClawState {
        //Claw shoot cube
        CLAW_OUTTAKE(
                -0.1,
                ClawControlMode.POSITION,
                0.295,
                ControlMode.PercentOutput,
                0.2
        ),
        //Claw shoot cube
        CLAW_OUTTAKE_HYBRID(
                -0.2,
                ClawControlMode.POSITION,
                .295,
                ControlMode.PercentOutput,
                .3
        ),
        //Claw tilted down and closed
        CLAW_HOLDING(
                0.2,
                ClawControlMode.POSITION,
                0,
                ControlMode.PercentOutput,
                -0.37
        ),
        //Claw tilted down and open cone
        CLAW_INTAKING_CONE(
                0.5,
                ClawControlMode.POSITION,
                0.31,
                ControlMode.Position,
                100
        ),
        //Claw tilted down and open cube
        CLAW_INTAKING_CUBE(
                0.5,
                ClawControlMode.POSITION,
                0.3,
                ControlMode.Position,
                700
        ),
        //Claw tilted down and standby wheel speed
        CLAW_STANDBY(
                0.2,
                ClawControlMode.POSITION,
                0,
                ControlMode.PercentOutput,
                -0.37
        ),
        //Drop claw to outtake height
        CLAW_DROP(
                0.3,
                ClawControlMode.POSITION,
                0.2,
                ControlMode.PercentOutput,
                -0.37
        ),
        CLAW_ANGLE_SHOOT(
                0.2,
                ClawControlMode.POSITION,
                0.12,
                ControlMode.PercentOutput,
                -0.37
        ),
        CLAW_SHOOT_HIGH(
                -0.8,
                ClawControlMode.POSITION,
                0.12,
                ControlMode.PercentOutput,
                -0.37
        ),
        CLAW_SHOOT_LOW(
                -0.3,
                ClawControlMode.POSITION,
                0.12,
                ControlMode.PercentOutput,
                -0.37
        ),
        CLAW_ANGLE_CUBE(
                0.6,
                ClawControlMode.POSITION,
                0.4,
                ControlMode.Position,
                700
        ),
        SINGLE_SUB(
                0.5,
                ClawControlMode.POSITION,
                0.2,
                ControlMode.Position,
                200
        ),
        TIPPED_CONE(
                0.5,
                ClawControlMode.POSITION,
                0.45,
                ControlMode.Position,
                100
        );

        final double intakeWheelsPercentOutput;
        final ClawControlMode clawControlMode;
        final double tiltPositionRots;
        final ControlMode openCloseControlMode;
        final double openCloseRots;

        public double getIntakeWheelsPercentOutput() {
            return intakeWheelsPercentOutput;
        }

        public ClawControlMode getClawControlMode() {
            return clawControlMode;
        }

        public double getTiltPositionRots() {
            return tiltPositionRots;
        }

        public ControlMode getOpenCloseControlMode() {
            return openCloseControlMode;
        }

        public double getOpenCloseRots() {
            return openCloseRots;
        }

        ClawState(final double intakeWheelsPercentOutput,
                  final ClawControlMode clawControlMode,
                  final double tiltPositionRots,
                  final ControlMode openCloseControlMode,
                  final double openCloseRots
        ) {
            this.intakeWheelsPercentOutput = intakeWheelsPercentOutput;
            this.clawControlMode = clawControlMode;
            this.tiltPositionRots = tiltPositionRots;
            this.openCloseControlMode = openCloseControlMode;
            this.openCloseRots = openCloseRots;
        }
    }

    public enum CANdleState {
        OFF,
        YELLOW,
        PURPLE
    }

    public enum DriverProfiles {
        DRIVER1,
        DRIVER2
    }

    public enum SwerveSpeeds {
        FAST,
        NORMAL,
        SLOW,
    }

    public enum GridPositions {
        LEFT,
        CENTER,
        RIGHT,
    }
}
