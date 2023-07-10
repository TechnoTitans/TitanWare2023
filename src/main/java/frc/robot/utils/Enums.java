package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Enums {
    public enum ElevatorState {
        ELEVATOR_RESET("Reset"), //DutyCycle back to limit switch to reset encoder
        ELEVATOR_EXTENDED_HIGH("ExtendedHigh"), //Elevator High and Horizontal extended
        ELEVATOR_EXTENDED_MID("ExtendedMid"), //Elevator Mid and Horizontal extended
        ELEVATOR_EXTENDED_PLATFORM("DoubleSubstation"), //Elevator Platform and Horizontal extended
        ELEVATOR_STANDBY("Standby"), //Elevator at normal height
        ELEVATOR_CUBE("Cube"),
        SINGLE_SUB("SingleSubstation"),
        ELEVATOR_TIPPED_CONE("TippedCone");

        final String name;
        ElevatorState(final String name) {
            this.name = name;
        }
    }

    public enum ElevatorMode {
        POSITION("Position"),
        MOTION_MAGIC("MotionMagic"),
        DUTY_CYCLE("DutyCycle");

        final String name;
        ElevatorMode(final String name) {
            this.name = name;
        }
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
