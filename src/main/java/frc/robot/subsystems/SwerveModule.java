package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.*;
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

    public SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder turnEncoder, double magnetOffset) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        config();
    }

    private void config() {
        //TODO MAKE SURE SENSORS ARE CCW+ AND GYRO IS CCW+
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfigs.MagnetOffset = -magnetOffset;

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor = magnetSensorConfigs;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.2;

        TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.Slot0.kP = 0.1;
        driverConfig.Slot0.kI = 0.002;
        driverConfig.Slot0.kD = 5;
        driverConfig.Slot0.kS = 0.045;
        driverConfig.ClosedLoopRamps = closedLoopRampsConfigs;
        driverConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driverConfig);

        TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Slot0.kP = 0.47;
        turnerConfig.Slot0.kI = 0;
        turnerConfig.Slot0.kD = 0;
//        turnerConfig.closedloopRamp = 0.1;
//        turnerConfig.neutralDeadband = 0.07;
        turnerConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
        turnerConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
        turnerConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnMotor.getConfigurator().apply(turnerConfig);
    }

    public double getAngle() {
        return turnEncoder.getAbsolutePosition().getValue();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValue() / Constants.Modules.DRIVER_TICKS_PER_WHEEL_RADIAN * Constants.Modules.WHEEL_RADIUS * 10, Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValue() * ((Constants.Modules.WHEEL_RADIUS*2*Math.PI) / (8.14 * 2048.0)), Rotation2d.fromDegrees(getAngle()));
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentWheelRotation = Rotation2d.fromDegrees(getAngle()); // TODO: CHECK THIS
        SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        double desired_driver_velocity_rps = wantedState.speedMetersPerSecond / Constants.Modules.WHEEL_RADIUS;

        Rotation2d delta_rotation = currentWheelRotation.minus(wantedState.angle);

        VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(desired_driver_velocity_rps, true, 0, 0, false);
        PositionDutyCycle positionDutyCycle = new PositionDutyCycle(delta_rotation.getRotations(), true, 0, 0, false);

        driveMotor.setControl(velocityDutyCycle);
        turnMotor.setControl(positionDutyCycle);
    }

    public void percentOutputControl(double output) {
        driveMotor.set(output);
        turnMotor.set(0);
    }

    public void manualVelocityControl(double rotationPerSecond) {
        VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(rotationPerSecond, true, 0, 0, false);
        driveMotor.setControl(velocityDutyCycle);
        turnMotor.set(0);
    }

    public void brake() {
        MotorOutputConfigs brakeConfig = new MotorOutputConfigs();
        brakeConfig.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(brakeConfig);
    }

    public void coast() {
        MotorOutputConfigs brakeConfig = new MotorOutputConfigs();
        brakeConfig.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(brakeConfig);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
