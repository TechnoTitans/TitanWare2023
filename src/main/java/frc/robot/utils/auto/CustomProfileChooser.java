package frc.robot.utils.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class CustomProfileChooser<V extends Enum<V>> implements LoggedDashboardInput, AutoCloseable {
    private final String ntTableName;

    private final StringArrayPublisher profilesPublisher;
    private final StringSubscriber selectedProfileSubscriber;
    private final LinkedHashMap<String, V> profileMap;
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(ntTableName, selectedProfile);
        }

        public void fromLog(LogTable table) {
            selectedProfile = table.getString(ntTableName, selectedProfile);
        }
    };

    private String selectedProfile;

    public CustomProfileChooser(
            final String ntTableName,
            final String ntPubName,
            final String ntSubName,
            final V defaultProfile
    ) {
        this.ntTableName = ntTableName;
        this.selectedProfile = defaultProfile.name();

        final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        this.profilesPublisher = ntTable.getStringArrayTopic(ntPubName).publish();
        this.selectedProfileSubscriber = ntTable.getStringTopic(ntSubName).subscribe(selectedProfile);
        this.profileMap = new LinkedHashMap<>(Map.of(selectedProfile, defaultProfile));

        Logger.getInstance().registerDashboardInput(this);
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
        return profileMap.get(selectedProfile);
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

    @Override
    public void periodic() {
        if (!Logger.getInstance().hasReplaySource()) {
            selectedProfile = selectedProfileSubscriber.get();
        }
        Logger.getInstance().processInputs(prefix, inputs);
    }
}
