package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Enums;
import frc.robot.utils.MathMethods;

@SuppressWarnings("unused")
public class Limelight {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Debugging booleans
    boolean targetFound = false;
    boolean targetAligned = false;

    // Data grabbing
    private final IntegerSubscriber tv = table.getIntegerTopic("tv").subscribe(0); // has any targets (0 or 1)
    private final DoubleSubscriber tx = table.getDoubleTopic("tx").subscribe(0.0); // horizontal offset from crosshair to target
    private final DoubleSubscriber ty = table.getDoubleTopic("ty").subscribe(0.0); // vertical offset from crosshair to target
    private final DoubleSubscriber ta = table.getDoubleTopic("ta").subscribe(0.0); // Target area 0% to 100% of image

    private final DoublePublisher ledMode = table.getDoubleTopic("ledMode").publish();
    public Limelight() {}

    public double calculateDistance() {
        // Hardware constants
        // TODO: find these measurements, won't really need
        final double HEIGHT_FROM_FLOOR_GOAL = 24.0; //TODO:
        final double HEIGHT_FROM_FLOOR_CAMERA = 21;
        final double ANGLE_FROM_FLOOR = 20;
        return Math.max((HEIGHT_FROM_FLOOR_GOAL - HEIGHT_FROM_FLOOR_CAMERA) / Math.tan(Math.toRadians(ANGLE_FROM_FLOOR + ty.getAsDouble()))-7.9, 0);
    }


    public boolean withinDeadband(double error) {
        double ERROR_TOLERANCE = 1;
        return MathMethods.withinRange(error, 0, ERROR_TOLERANCE);
    }

//    public void setOFFSET(double OFFSET) {
//        this.OFFSET = OFFSET;
//    }

    public double getSwerveError() {
        return tx.getAsDouble();
    }

    public double getX() {
        return tx.getAsDouble();
    }

    public double getY() {
        return ty.getAsDouble();
    }

    public boolean isTargetAligned() {
        return withinDeadband(tx.getAsDouble());
    }

    public boolean isTargetFound() {
        return tv.getAsLong() > 0;
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

