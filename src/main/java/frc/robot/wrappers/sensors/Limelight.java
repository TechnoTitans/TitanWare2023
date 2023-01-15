package frc.robot.wrappers.sensors;

import edu.wpi.first.networktables.*;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;

@SuppressWarnings("unused")
public class Limelight {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double targetVelocity = 0;
    private double turretError = 0;

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

        // Calculate velocity and turret angle
        targetVelocity = calculateVelocity(distance);
        double OFFSET = -2;
        turretError = xError + OFFSET;
    }

    public double calculateDistance(double angle) {
        // Hardware constants
        // TODO: find these measurements
        double HEIGHT_FROM_FLOOR_GOAL = 64.5;
        double HEIGHT_FROM_FLOOR_CAMERA = 6;
        double ANGLE_FROM_FLOOR = 0;
        return (HEIGHT_FROM_FLOOR_GOAL - HEIGHT_FROM_FLOOR_CAMERA) / Math.tan(Math.toRadians(ANGLE_FROM_FLOOR + angle));
    }

    public double calculateVelocity(double distance) {
        //TODO: CALCULATE EQUATION
        return .0172 * Math.pow(distance, 3) + 5.4876 * Math.pow(distance, 2) + 584.54 * distance + 23128;
    }

    public boolean withinDeadband(double error) {
        double ERROR_TOLERANCE = 1;
        return MathMethods.withinRange(error, 0, ERROR_TOLERANCE);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTurretError() {
        return turretError;
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

