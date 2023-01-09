package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.profiler.Profiler;
import frc.robot.wrappers.motors.TitanFX;


@SuppressWarnings({"unused", "ConstantConditions"})
public class TankDrive extends SubsystemBase {
    public static final double DRIVETRAIN_INCHES_PER_PULSE = (Constants.WHEEL_DIAMETER * Math.PI) / (Constants.ENCODER_EDGES_PER_REV * Constants.GEARING); // inches per pulse
    public static final double MAX_MOTOR_TEMP = 70;

    public static boolean SHIFT_HIGH_TORQUE = true;
    public static boolean SHIFT_LOW_TORQUE = !SHIFT_HIGH_TORQUE;

    private final TitanFX left, right;
    private final Pigeon2 gyro;
    private final Solenoid shifterSolenoid;
    private final Compressor compressor;

    //robot characterization value
    public static final double kPDriveVel = 8.5;

    public static final double THROTTLE_SENSITIVITY_DEFAULT = Profiler.getProfile().ThrottleSensitivity;
    public static final double STEERING_SENSITIVITY_DEFAULT = Profiler.getProfile().SteeringSensitivity;

    private final DifferentialDriveOdometry odometry;

    //TankDrive setup
    private final PIDController drivePID;

    public TankDrive(TitanFX left, TitanFX right, Pigeon2 gyro, Solenoid shifterSolenoid, Compressor compressor) {
        this.left = left;
        this.right = right;
        this.gyro = gyro;
        this.shifterSolenoid = shifterSolenoid;
        this.drivePID = new PIDController(0, 0, 0);
        this.compressor = compressor;
        resetEncoders();

        this.odometry = new DifferentialDriveOdometry(getHeading(), 0, 0);
    }


    //Odometry Stuff
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Gear Position", shifterSolenoid.get());
        odometry.update(getHeading(), left.getSensorCollection().getIntegratedSensorPosition() * DRIVETRAIN_INCHES_PER_PULSE * 0.0254, -right.getSensorCollection().getIntegratedSensorPosition() * DRIVETRAIN_INCHES_PER_PULSE * 0.0254);
    }

    //returns currently estimated pose of robot
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(left.getSensorCollection().getIntegratedSensorVelocity() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254, -right.getSensorCollection().getIntegratedSensorVelocity() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254);
    }

    public void resetOdometry(Pose2d pose){
        gyro.setYaw(0);
        resetEncoders();
        odometry.resetPosition(getHeading(), 0, 0, pose);
    }
    //controls left and right sides directly with voltages
    public void tankDriveVolts(double leftVolts, double rightVolts){
        getLeft().setVoltage(leftVolts);
        getRight().setVoltage(rightVolts);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public void reset() {
        odometry.resetPosition(getHeading(),0, 0, odometry.getPoseMeters());
    }

    public PIDController getPID() {
        return drivePID;
    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
    }

    public void set(double leftTSpeed, double rightTSpeed) {
        left.set(leftTSpeed);
        right.set(rightTSpeed);
    }

    public void setRPM(double leftTRPM, double rightTRPM) {
        left.setVelocityPID(leftTRPM);
        right.setVelocityPID(rightTRPM);
    }

    public void stop() {
        this.set(0);
    }

    public void brake() {
        left.brake();
        right.brake();
    }

    public void coast() {
        left.coast();
        right.coast();
    }

    public void resetEncoders() {
        this.left.getSensorCollection().setIntegratedSensorPosition(0, 0);
        this.right.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public double getAverageEncoderDistance() {
        return ((left.getSensorCollection().getIntegratedSensorPosition() + right.getSensorCollection().getIntegratedSensorPosition()) / 2.0);
    }

    public TitanFX getLeft() {
        return left;
    }

    public TitanFX getRight() {
        return right;
    }

    public void enableBrownoutProtection() {
        left.enableBrownoutProtection();
        right.enableBrownoutProtection();
    }

    public void disableBrownoutProtection() {
        left.disableBrownoutProtection();
        right.disableBrownoutProtection();
    }

    public double[] getSpeed() {
        return new double[]{left.getSpeed(), right.getSpeed()};
    }


    public Pigeon2 getPigeon() {
        return gyro;
    }

    public boolean getPressureSwitchValue() {
        return this.compressor.getPressureSwitchValue();
    }

    public boolean getShifterEnabled() {
        return this.shifterSolenoid.get();
    }

    public void setShifter(boolean pistonDeployed) {
        this.shifterSolenoid.set(pistonDeployed);
    }

    public void toggleShifter() {
        boolean currentStatus = this.getShifterEnabled();
        this.setShifter(!currentStatus);
    }

    public RamseteCommand createRamseteCommand(Trajectory trajectory, TankDrive driveTrain) {
        return new RamseteCommand(
                trajectory,
                driveTrain::getPose,
                new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA),
                Constants.kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
                new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
                driveTrain::tankDriveVolts,
                driveTrain
        );
    }

}

