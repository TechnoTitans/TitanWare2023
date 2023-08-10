package frc.robot.utils.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.Constants;
import frc.robot.utils.closeables.ToClose;

import java.util.*;
import java.util.function.Function;

public class CustomAutoChooser<I, V extends AutoOption> implements AutoCloseable {
    private final StringArrayPublisher autoPublisher;
    private final StringSubscriber selectedAutoSubscriber;
    private final LinkedHashMap<String, V> autoMap;

    private final HashSet<V> ignoredSet = new HashSet<>();

    public CustomAutoChooser(
            final String ntTableName,
            final String ntPubName,
            final String ntSubName,
            final V defaultAuto
    ) {
        final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        this.autoPublisher = ntTable.getStringArrayTopic(ntPubName).publish();
        this.selectedAutoSubscriber = ntTable.getStringTopic(ntSubName).subscribe(defaultAuto.pathName());
        this.autoMap = new LinkedHashMap<>(Map.of(defaultAuto.pathName(), defaultAuto));

        ToClose.add(this);
    }

    private boolean shouldIgnoreOption(final V object) {
        return ignoredSet.contains(object) || !object.hasCompetitionType(Constants.CURRENT_COMPETITION_TYPE);
    }

    public void addOption(final String name, final V object) {
        if (shouldIgnoreOption(object)) {
            ignoredSet.add(object);
        } else {
            autoMap.put(name, object);
            autoPublisher.set(autoMap.keySet().toArray(String[]::new));
        }
    }

    /**
     * Adds an {@link AutoOption} object to the by using the name of the option
     * @param object the {@link AutoOption} or V
     * @see AutoOption
     * @see CustomAutoChooser#addOption(String, AutoOption)
     */
    public void addAutoOption(final V object) {
        addOption(object.pathName(), object);
    }

    public V getSelected() {
        return autoMap.get(selectedAutoSubscriber.get());
    }

    public void addOptionsIfNotPresent(
            final Function<V, String> nameFunction, final Function<I, V> objectFunction, final List<I> inputs
    ) {
        for (final I input : inputs) {
            final V computedObject = objectFunction.apply(input);
            if (!autoMap.containsValue(computedObject)) {
                addOption(nameFunction.apply(computedObject), computedObject);
            }
        }
    }

    @Override
    public void close() {
        selectedAutoSubscriber.close();
        autoPublisher.close();
    }
}
