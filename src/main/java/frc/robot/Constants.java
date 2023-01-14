package frc.robot;

@SuppressWarnings("unused")
public interface Constants {
    interface Modules {
        double WHEEL_RADIUS = 0.0508; //2 in
        int MOTOR_ROTATION_TO_TALON_ENCODER_TICKS = 2048;
        int CANCODER_TICKS_PER_ROTATION = 4096;
        double DRIVER_GEAR_RATIO = 8.14;
        double TURNER_GEAR_RATIO = 150/7d;
        double TICKS_PER_MOTOR_RADIAN = MOTOR_ROTATION_TO_TALON_ENCODER_TICKS / (2 * Math.PI);
        double TICKS_PER_DRIVER_WHEEL_ROTATION = MOTOR_ROTATION_TO_TALON_ENCODER_TICKS * DRIVER_GEAR_RATIO;
        double DRIVER_TICKS_PER_WHEEL_RADIAN = TICKS_PER_MOTOR_RADIAN * DRIVER_GEAR_RATIO;
        int HUNDREDMILLISECONDS_TO_1SECOND = 10;
        double ONESECOND_TO_100_MILLISECONDS = .1;
        double TICKS_PER_TALON_ENCODER_DEGREE = MOTOR_ROTATION_TO_TALON_ENCODER_TICKS / 360.0;
        double TICKS_PER_CANCODER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360.0;
    }

    interface Swerve {
        double ROBOT_MAX_SPEED = 13.5; //TODO: tune this
        double MODULE_MAX_SPEED = 9;
        double ROBOT_MAX_ANGULAR_SPEED = Math.PI;
        double TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        double TELEOP_MAX_ANGULAR_SPEED = Math.PI;
        double TRAJ_MAX_SPEED = ROBOT_MAX_SPEED;
        double TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED * 0.5;
        double TRAJ_MAX_ANGULAR_SPEED = ROBOT_MAX_ANGULAR_SPEED;
        double TRAJ_MAX_ANGULAR_ACCELERATION = Math.PI;
        double ROTATE_P = 2; //TUNE THIS: (rotation pid) 2
        double AUTO_BALANCE_PITCH_P = 0.005; // P value for auto balance
    }
}