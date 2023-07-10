package frc.robot.subsystems.elevator;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.utils.Enums;
import frc.robot.utils.SimUtils;
import frc.robot.wrappers.motors.TitanMAX;

//TODO: literally all of the elevator sim stuff
public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSimSolver elevatorSimSolver;

    private final TalonFX verticalElevatorMotor, verticalElevatorMotorFollower;
    private final DCMotorSim verticalElevatorSimMotors;
    private final InvertedValue verticalElevatorMotorR, verticalElevatorMotorFollowerR;
    private final SensorDirectionValue verticalElevatorEncoderR;

    private final TitanMAX horizontalElevatorMotor;
    private final DCMotorSim horizontalElevatorSimMotor;

    private final CANcoder verticalElevatorEncoder, horizontalElevatorEncoder;
    private final DigitalInput verticalElevatorLimitSwitch, horizontalElevatorLimitSwitch, elevatorHorizontalHighLimitSwitch;

    private Enums.ElevatorState desiredState = Enums.ElevatorState.ELEVATOR_RESET;
    private final ProfiledPIDController horizontalElevatorPID;

    private final PositionVoltage positionVoltage;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycleOut;
    private Enums.ElevatorMode verticalElevatorMode;

    private double
            HEPositionRotations = 0, //Horizontal Elevator Target Rotations
            VEPositionRotations = 0; //Vertical Elevator Target Rotations

    private boolean VESwitchFlag = false;
    private boolean horizontalPositionalControl = false;

    public ElevatorIOSim(
            final TalonFX verticalElevatorMotor,
            final InvertedValue verticalElevatorMotorR,
            final TalonFX verticalElevatorMotorFollower,
            final InvertedValue verticalElevatorMotorFollowerR,
            final CANcoder verticalElevatorEncoder,
            final CANcoder horizontalElevatorEncoder,
            final SensorDirectionValue verticalElevatorEncoderR,
            final TitanMAX horizontalElevatorMotor,
            final DigitalInput verticalElevatorLimitSwitch,
            final DigitalInput horizontalElevatorLimitSwitch,
            final DigitalInput elevatorHorizontalHighLimitSwitch) {
        this.elevatorSimSolver = new ElevatorSimSolver();

        this.verticalElevatorMotor = verticalElevatorMotor;
        this.verticalElevatorMotorR = verticalElevatorMotorR;
        this.verticalElevatorMotorFollower = verticalElevatorMotorFollower;
        this.verticalElevatorMotorFollowerR = verticalElevatorMotorFollowerR;
        this.verticalElevatorEncoder = verticalElevatorEncoder;
        this.verticalElevatorEncoderR = verticalElevatorEncoderR;

        this.verticalElevatorSimMotors = new DCMotorSim(
                DCMotor.getFalcon500(2),
                0.0938,
                Constants.Sim.ELEVATOR_VERTICAL_EXT_MOI
        );

        SimUtils.setCTRESimStateMotorInverted(verticalElevatorMotor, verticalElevatorMotorR);
        SimUtils.setCTRESimStateMotorInverted(verticalElevatorMotorFollower, verticalElevatorMotorFollowerR);

        this.horizontalElevatorMotor = horizontalElevatorMotor;
        this.horizontalElevatorEncoder = horizontalElevatorEncoder;

        //TODO: get real gearing and moi
        this.horizontalElevatorSimMotor = new DCMotorSim(
                DCMotor.getNEO(1),
                1,
                1
        );

        this.verticalElevatorLimitSwitch = verticalElevatorLimitSwitch;
        this.horizontalElevatorLimitSwitch = horizontalElevatorLimitSwitch;
        this.elevatorHorizontalHighLimitSwitch = elevatorHorizontalHighLimitSwitch;

        config();

        this.horizontalElevatorPID = new ProfiledPIDController(0.3, 0, 0,
                new TrapezoidProfile.Constraints(10, 20));

        this.positionVoltage = new PositionVoltage(
                0, true, 0, 0, false);
        this.motionMagicVoltage = new MotionMagicVoltage(
                0, true, 0, 0, false);
        this.dutyCycleOut = new DutyCycleOut(
                0, true, false);
    }

    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        elevatorSimSolver.update(1, 1);
    }

    @Override
    public void config() {

    }

    @Override
    public void setDesiredState(final Enums.ElevatorState state) {

    }

    @Override
    public Enums.ElevatorState getDesiredState() {
        return Enums.ElevatorState.ELEVATOR_RESET;
    }
}
