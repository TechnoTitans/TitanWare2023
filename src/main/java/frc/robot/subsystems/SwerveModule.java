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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.motors.TitanFX;

@SuppressWarnings("unused")
public class SwerveModule extends SubsystemBase {
    private final TitanFX driveMotor, turnMotor;
    private final CANCoder turnEncoder;
    private final double magnetOffset;

    public SwerveModule(TitanFX driveMotor, TitanFX turnMotor, CANCoder turnEncoder, double magnetOffset) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        config();
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

        driveMotor.configFactoryDefault();
        TalonFXConfiguration driverConfig = new TalonFXConfiguration();
        driverConfig.slot0.kP = 0.1;
        driverConfig.slot0.kI = 0.002;
        driverConfig.slot0.integralZone = 200;
        driverConfig.slot0.kD = 5;
        driverConfig.slot0.kF = 0.045;
        driverConfig.closedloopRamp = 0.2;
        driveMotor.configAllSettings(driverConfig);
        driveMotor.setNeutralMode(NeutralMode.Coast);

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
        turnMotor.setNeutralMode(NeutralMode.Coast);
    }

    public double getAngle() {
        return turnEncoder.getAbsolutePosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() / Constants.Modules.DRIVER_TICKS_PER_WHEEL_RADIAN * Constants.Modules.WHEEL_RADIUS * 10, Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() * ((Constants.Modules.WHEEL_RADIUS*2*Math.PI) / (8.14 * 2048.0)), Rotation2d.fromDegrees(getAngle()));
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentWheelRotation = Rotation2d.fromDegrees(getAngle()); // TODO: CHECK THIS
        SwerveModuleState wantedState = SwerveModuleState.optimize(state, currentWheelRotation);
        double desired_driver_velocity_ticks = wantedState.speedMetersPerSecond / Constants.Modules.WHEEL_RADIUS * Constants.Modules.DRIVER_TICKS_PER_WHEEL_RADIAN * Constants.Modules.ONESECOND_TO_100_MILLISECONDS;
        Rotation2d delta_rotation = currentWheelRotation.minus(wantedState.angle);
        double delta_ticks = delta_rotation.getDegrees() * Constants.Modules.TICKS_PER_CANCODER_DEGREE;
        double current_ticks = turnMotor.getSelectedSensorPosition();
        double desired_turner_ticks = current_ticks + delta_ticks;

        driveMotor.set(TalonFXControlMode.Velocity, desired_driver_velocity_ticks);
        turnMotor.set(TalonFXControlMode.Position, desired_turner_ticks);
    }

    public void percentOutputControl(double output) {
        driveMotor.set(TalonFXControlMode.PercentOutput, output);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void manualVelocityControl(double velocity_ticks_per_100ms) {
        driveMotor.set(TalonFXControlMode.Velocity, velocity_ticks_per_100ms);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void brake() {
        driveMotor.brake();
    }

    public void coast() {
        driveMotor.coast();
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.Velocity, 0);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}
