package frc.robot.wrappers.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MathMethods;

@SuppressWarnings("unused")
public class Limelight {
    private double targetVelocity = 0;
    private double turretError = 0;

    public double xError;
    public double yError;
    public double distance;

    // Debugging booleans
    boolean targetFound = false;
    boolean targetAligned = false;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight() {
    }

    public void periodic() {
        // Data grabbing
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        // Read data
        xError = tx.getDouble(0);
        yError = ty.getDouble(0);
        distance = calculateDistance(yError);

        // Check if target is found
        targetFound = tv.getDouble(0.0) > 0;
        targetAligned = withinDeadband(xError);

        // Calculate velocity and turret angle
        targetVelocity = calculateVelocity(distance);
        double OFFSET = -2;
        turretError = xError + OFFSET;

        SmartDashboard.putNumber("Distance from target", distance);

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

    public void disableLEDs() {
        table.getEntry("ledMode").setNumber(1);
    }

    public void enableLEDs() {
        table.getEntry("ledMode").setNumber(3);
    }

    public void configLEDs() {
        table.getEntry("ledMode").setNumber(0);
    }

    public void LEDMode(boolean mode) {
        if (mode) {
            table.getEntry("ledMode").setNumber(3);
        } else {
            table.getEntry("ledMode").setNumber(1);
        }
    }
}

