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
    private final VelocityDutyCycle velocityDutyCycle;
    private final PositionDutyCycle positionDutyCycle;

    public SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder turnEncoder, double magnetOffset) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;

        velocityDutyCycle = new VelocityDutyCycle(0, true, 0, 0, false);
        positionDutyCycle = new PositionDutyCycle(0, true, 0, 0, false);

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

        TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.Slot0.kP = 0.3;
//        driverConfig.Slot0.kI = 0.002;
//        driverConfig.Slot0.kD = 5;
        driverConfig.Slot0.kD = 0;
//        driverConfig.Slot0.kS = 0.045;
        driverConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        driverConfig.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driverConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driverConfig);

        TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Slot0.kP = 0.1; //need
        turnerConfig.Slot0.kI = 0; //need
        turnerConfig.Slot0.kD = 0; //need
//        turnerConfig.closedloopRamp = 0.1;
//        turnerConfig.neutralDeadband = 0.07;
        turnerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.07;
        turnerConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
        turnerConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
        turnerConfig.ClosedLoopGeneral.ContinuousWrap = true; //need
        turnerConfig.Feedback.SensorToMechanismRatio = Constants.Modules.TURNER_GEAR_RATIO; //need / correct
        turnerConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID(); //need
        turnerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // need idk what dif is betwee   n FusedCancoder
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; //need
        turnMotor.getConfigurator().apply(turnerConfig);
    }

    public double getAngle() {
        return turnEncoder.getAbsolutePosition().getValue();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValue() * (2*Constants.Modules.WHEEL_RADIUS*Math.PI), Rotation2d.fromDegrees(getAngle()));
//        return new SwerveModuleState(driveMotor.getVelocity().getValue() / Constants.Modules.DRIVER_TICKS_PER_WHEEL_RADIAN * Constants.Modules.WHEEL_RADIUS * 10, Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValue() * (2*Constants.Modules.WHEEL_RADIUS*Math.PI), Rotation2d.fromDegrees(getAngle()));
//        return new SwerveModulePosition(driveMotor.getPosition().getValue() * ((Constants.Modules.WHEEL_RADIUS*2*Math.PI) / (8.14 * 2048.0)), Rotation2d.fromDegrees(getAngle()));
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentWheelRotation = Rotation2d.fromDegrees(getAngle());
        SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        double desired_driver_velocity_rps = wantedState.speedMetersPerSecond / (2 * Math.PI * Constants.Modules.WHEEL_RADIUS);

        Rotation2d delta_rotation = currentWheelRotation.minus(wantedState.angle);

        driveMotor.setControl(velocityDutyCycle.withVelocity(desired_driver_velocity_rps));
        turnMotor.setControl(positionDutyCycle.withPosition(delta_rotation.getRotations()));
    }

    public void percentOutputControl(double output) {
        driveMotor.set(output);
        turnMotor.stopMotor();
    }

    public void manualVelocityControl(double rotationPerSecond) {
        driveMotor.setControl(velocityDutyCycle.withVelocity(rotationPerSecond));
        turnMotor.stopMotor();
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
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
