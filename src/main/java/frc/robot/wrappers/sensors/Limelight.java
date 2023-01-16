package frc.robot.wrappers.sensors;

import edu.wpi.first.networktables.*;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;

@SuppressWarnings("unused")
public class Limelight {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double swerveError = 0;

    public double xError;
    public double yError;
    public double distance;

    // Debugging booleans
    boolean targetFound = false;
    boolean targetAligned = false;

    // Data grabbing
    private final IntegerSubscriber tv = table.getIntegerTopic("tv").subscribe(0); // has any targets (0 or 1)
    private final DoubleSubscriber tx = table.getDoubleTopic("tx").subscribe(0.0); // horizontal offset from crosshair to target
    private final DoubleSubscriber ty = table.getDoubleTopic("ty").subscribe(0.0); // vertical offset from crosshair to target
    private final DoubleSubscriber ta = table.getDoubleTopic("ta").subscribe(0.0); // Target area 0% to 100% of image

    private final IntegerPublisher ledMode = table.getIntegerTopic("ledMode").publish();

    public Limelight() {
    }

    public void periodic() {
        // Read data
        xError = tx.getAsDouble();
        yError = ty.getAsDouble();
        distance = calculateDistance(yError);

        // Check if target is found
        targetFound = tv.get() > 0;
        targetAligned = withinDeadband(xError);

        double OFFSET = 0; //TODO: fientune during testing
        swerveError = xError + OFFSET;
    }

    public double calculateDistance(double angle) {
        // Hardware constants
        // TODO: find these measurements, won't really need
        double HEIGHT_FROM_FLOOR_GOAL = 64.5; //TODO:
        double HEIGHT_FROM_FLOOR_CAMERA = 6;
        double ANGLE_FROM_FLOOR = 0;
        return (HEIGHT_FROM_FLOOR_GOAL - HEIGHT_FROM_FLOOR_CAMERA) / Math.tan(Math.toRadians(ANGLE_FROM_FLOOR + angle));
    }


    public boolean withinDeadband(double error) {
        double ERROR_TOLERANCE = 1;
        return MathMethods.withinRange(error, 0, ERROR_TOLERANCE);
    }


    public double getSwerveError() {
        return swerveError;
    }

    public boolean isTargetAligned() {
        return targetAligned;
    }

    public boolean isTargetFound() {
        return targetFound;
    }

    public void setLEDMode(Enums.LimeLightLEDState limeLightLEDState) {
        switch (limeLightLEDState) {
            case LED_OFF:
                ledMode.set(1);
                break;
            case LED_CONFIG:
                ledMode.set(0);
                break;
            case LED_ON:
                ledMode.set(3);
                break;
        }
    }
}

