package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.MagnetSensorConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor, turnMotor;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final VelocityDutyCycle driveDutyCycle;
    private final PositionDutyCycle turnDutyCycle;

    public SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder turnEncoder, double magnetOffset,
                        boolean invertedDrive, boolean invertedTurn) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;

        this.driveDutyCycle = new VelocityDutyCycle(0, true, 0, 0, false);
        this.turnDutyCycle = new PositionDutyCycle(0, true, 0, 0, false);

        config();
    }

    private void config() {
        //TODO MAKE SURE SENSORS ARE CCW+ AND GYRO IS CCW+
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.MagnetOffset = -magnetOffset;
        //TODO SPIN EACH WHEEL CCW AND MAKE SURE IT IS +
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor = magnetSensorConfigs;
        canCoderConfiguration.FutureProofConfigs = true;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.Slot0.kP = 0.1;
        driverConfig.Slot0.kI = 0.002;
        driverConfig.Slot0.kD = 5;
//        driverConfig.Slot0.kS = 0.045;
        driverConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        driverConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driverConfig.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driveMotor.getConfigurator().apply(driverConfig);

        TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Slot0.kP = 0.47;
        turnerConfig.Slot0.kI = 0;
        turnerConfig.Slot0.kD = 0;
//        turnerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
//        turnerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.07;
        turnerConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
        turnerConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
        turnerConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnerConfig.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnMotor.getConfigurator().apply(turnerConfig);
    }

    public double getRotations() {
        return turnEncoder.getAbsolutePosition().getValue();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValue() * (2*Math.PI*Constants.Modules.WHEEL_RADIUS), Rotation2d.fromRotations(getRotations()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValue() * ((Constants.Modules.WHEEL_RADIUS*2*Math.PI)), Rotation2d.fromRotations(getRotations()));
        // if position isnt scaled use this
//        return new SwerveModulePosition(driveMotor.getPosition().getValue() * ((Constants.Modules.WHEEL_RADIUS*2*Math.PI) / 8.14), Rotation2d.fromRotations(getRotations()));
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentWheelRotation = Rotation2d.fromRotations(getRotations()); // TODO: CHECK THIS
        SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        double desired_driver_velocity = wantedState.speedMetersPerSecond / (2*Math.PI*Constants.Modules.WHEEL_RADIUS);

        driveMotor.setControl(driveDutyCycle.withVelocity(desired_driver_velocity));
        turnMotor.setControl(turnDutyCycle.withPosition(wantedState.angle.getRotations()));
    }

    public void percentOutputControl(double output) {
        driveMotor.set(output);
        turnMotor.set(0);
    }

    public void manualVelocityControl(double velocity) {
        driveMotor.setControl(driveDutyCycle.withVelocity(velocity));
        turnMotor.set(0);
    }

    public void brake() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(motorOutputConfigs);
    }

    public void coast() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(motorOutputConfigs);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
