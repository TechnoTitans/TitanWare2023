package frc.robot.utils;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class TitanBoard implements Runnable {
    private static Notifier titanBoardThread;

    private static final NetworkTable robotNT = NetworkTableInstance.getDefault().getTable("TitanBoard");
    private static final LinkedHashMap<GenericPublisher, Supplier<?>> primitivePublisherSupplierMap = new LinkedHashMap<>();

    private static final double[] precomputedSwerveModuleOffsets = new double[] {
            Constants.Swerve.FL_OFFSET.getX(),
            Constants.Swerve.FL_OFFSET.getY(),
            Constants.Swerve.FR_OFFSET.getX(),
            Constants.Swerve.FR_OFFSET.getY(),
            Constants.Swerve.BL_OFFSET.getX(),
            Constants.Swerve.BL_OFFSET.getY(),
            Constants.Swerve.BR_OFFSET.getX(),
            Constants.Swerve.BR_OFFSET.getY(),
    };

    private TitanBoard() {}

    public static void start() {
        if (titanBoardThread != null)
            return;

        titanBoardThread = new Notifier(new TitanBoard());
        titanBoardThread.setName("titanBoardThread");
        titanBoardThread.startPeriodic(0.05);
    }

    public static void addDouble(
            final String name,
            final Supplier<Double> doubleSupplier
    ) {
        primitivePublisherSupplierMap.put(robotNT.getTopic(name).genericPublish(
                NetworkTableType.kDouble.getValueStr()
        ), doubleSupplier);
    }

    public static void addDoubleArray(
            final String name,
            final Supplier<double[]> arraySupplier
    ) {
        primitivePublisherSupplierMap.put(robotNT.getTopic(name).genericPublish(
                NetworkTableType.kDoubleArray.getValueStr()
        ), arraySupplier);
    }

    public static void addInteger(
            final String name,
            final Supplier<Integer> integerSupplier
    ) {
        primitivePublisherSupplierMap.put(robotNT.getTopic(name).genericPublish(
                NetworkTableType.kInteger.getValueStr()
        ), integerSupplier);
    }

    public static void addString(
            final String name,
            final Supplier<String> stringSupplier
    ) {
        primitivePublisherSupplierMap.put(robotNT.getTopic(name).genericPublish(
                NetworkTableType.kString.getValueStr()
        ), stringSupplier);
    }

    public static void addBoolean(
            final String name,
            final Supplier<Boolean> booleanSupplier
    ) {
        primitivePublisherSupplierMap.put(robotNT.getTopic(name).genericPublish(
                NetworkTableType.kBoolean.getValueStr()
        ), booleanSupplier);
    }

    public static void addEncoder(
            final String name,
            final Supplier<Double> distance,
            final Supplier<Double> speed
    ) {
        addDouble(String.format("%s/distance", name), distance);
        addDouble(String.format("%s/speed", name), speed);
    }

    private static double[] makeDoubleArrayOfModuleStates(final SwerveModuleState[] swerveModuleStates) {
        final double[] moduleStatesDoubleRepresentation = new double[swerveModuleStates.length * 2];

        int representationIndex = 0;
        for (final SwerveModuleState state : swerveModuleStates) {
            moduleStatesDoubleRepresentation[representationIndex] = state.speedMetersPerSecond;
            moduleStatesDoubleRepresentation[representationIndex + 1] = state.angle.getRadians();

            representationIndex += 2;
        }

        return moduleStatesDoubleRepresentation;
    }

    @Override
    public void run() {
        for (final Map.Entry<GenericPublisher, Supplier<?>> entry : primitivePublisherSupplierMap.entrySet()) {
            entry.getKey().setValue(entry.getValue().get());
        }
    }
}
