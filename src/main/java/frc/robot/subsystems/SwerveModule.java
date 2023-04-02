package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.motors.TitanFX;

@SuppressWarnings("unused")
public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor;
    private final TitanFX turnMotor;
    private final CANCoder turnEncoder;
    private final double magnetOffset;
    private final InvertedValue invertedValue;
    private final VelocityVoltage velocityDutyCycle;

    public SwerveModule(TalonFX driveMotor, TitanFX turnMotor, CANCoder turnEncoder, InvertedValue invertedValue, double magnetOffset) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        this.invertedValue = invertedValue;

        config();

        this.velocityDutyCycle = new VelocityVoltage(0, true, 0, 0, false);
    }

    private void config() {
        //TODO MAKE SURE SENSORS ARE CCW+ AND GYRO IS CCW+
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.unitString = "deg";
        canCoderConfiguration.sensorDirection = false;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        canCoderConfiguration.magnetOffsetDegrees = -magnetOffset;
        turnEncoder.configAllSettings(canCoderConfiguration);

        com.ctre.phoenixpro.configs.TalonFXConfiguration driverConfig = new com.ctre.phoenixpro.configs.TalonFXConfiguration();
        driverConfig.Slot0.kP = 0.00060954;
        driverConfig.Slot0.kI = 0.002;
        driverConfig.Slot0.kD = 0.01;
        driverConfig.Slot0.kS = 0.25655;
        driverConfig.Slot0.kV = 2.9757;
        driverConfig.CurrentLimits.StatorCurrentLimit = 60;
        driverConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driverConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
        driverConfig.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driverConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driverConfig.MotorOutput.Inverted = invertedValue ;
        driveMotor.getConfigurator().apply(driverConfig);

        turnMotor.configFactoryDefault();
        TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.slot0.kP = 0.47;
        turnerConfig.slot0.kI = 0;
        turnerConfig.slot0.kD = 0;
        turnerConfig.slot0.kF = 0;
//        turnerConfig.closedloopRamp = 0.1;
//        turnerConfig.neutralDeadband = 0.07;
        turnerConfig.peakOutputForward = 0.5;
        turnerConfig.peakOutputReverse = -0.5;
        turnerConfig.remoteFilter0.remoteSensorDeviceID = turnEncoder.getDeviceID();
        turnerConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        turnerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        turnMotor.configAllSettings(turnerConfig);
        turnMotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getAngle() {
        return turnEncoder.getAbsolutePosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(-driveMotor.getVelocity().getValue() * (2*Math.PI*Constants.Modules.WHEEL_RADIUS), Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(-driveMotor.getPosition().getValue() * (2*Math.PI*Constants.Modules.WHEEL_RADIUS), Rotation2d.fromDegrees(getAngle()));
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentWheelRotation = Rotation2d.fromDegrees(getAngle()); // TODO: CHECK THIS
        SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        double desired_driver_velocity = wantedState.speedMetersPerSecond / (2 * Math.PI * Constants.Modules.WHEEL_RADIUS);
        Rotation2d delta_rotation = currentWheelRotation.minus(wantedState.angle);
        double delta_ticks = delta_rotation.getDegrees() * Constants.Modules.TICKS_PER_CANCODER_DEGREE;
        double current_ticks = turnMotor.getSelectedSensorPosition();
        double desired_turner_ticks = current_ticks + delta_ticks;

        driveMotor.setControl(velocityDutyCycle.withVelocity(desired_driver_velocity));
        turnMotor.set(TalonFXControlMode.Position, desired_turner_ticks);
    }

    public void percentOutputControl(double output) {
        driveMotor.set(output);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void manualVelocityControl(double rps) {
        driveMotor.setControl(velocityDutyCycle.withVelocity(rps));
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void brake() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().refresh(motorOutputConfigs);
    }

    public void coast() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().refresh(motorOutputConfigs);
    }
}
