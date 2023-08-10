package frc.robot.utils.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.utils.closeables.ToClose;

import java.util.*;
import java.util.function.Function;

public class CustomProfileChooser<V extends Enum<V>> implements AutoCloseable {
    private final StringArrayPublisher profilesPublisher;
    private final StringSubscriber selectedProfileSubscriber;
    private final LinkedHashMap<String, V> profileMap;

    public CustomProfileChooser(
            final String ntTableName,
            final String ntPubName,
            final String ntSubName,
            final V defaultProfile
    ) {
        final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        this.profilesPublisher = ntTable.getStringArrayTopic(ntPubName).publish();
        this.selectedProfileSubscriber = ntTable.getStringTopic(ntSubName).subscribe(defaultProfile.name());
        this.profileMap = new LinkedHashMap<>(Map.of(defaultProfile.name(), defaultProfile));

        ToClose.add(this);
    }

    public void addOption(final String name, final V object) {
        profileMap.put(name, object);
        profilesPublisher.set(profileMap.keySet().toArray(String[]::new));
    }

    /**
     * Adds an {@link AutoOption} object to the by using the name of the option
     * @param object the {@link AutoOption} or V
     * @see AutoOption
     * @see CustomAutoChooser#addOption(String, AutoOption)
     */
    public void addProfileOption(final V object) {
        addOption(object.name(), object);
    }

    public V getSelected() {
        return profileMap.get(selectedProfileSubscriber.get());
    }

    public void addOptionsIfNotPresent(
            final Function<V, String> nameFunction, final List<V> inputs
    ) {
        for (final V input : inputs) {
            if (!profileMap.containsValue(input)) {
                addOption(nameFunction.apply(input), input);
            }
        }
    }

    @Override
    public void close() {
        selectedProfileSubscriber.close();
        profilesPublisher.close();
    }
}
