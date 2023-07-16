package frc.robot.utils.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;

import java.util.HashSet;
import java.util.List;
import java.util.function.Function;

public class AutoChooser<I, V extends AutoOption> extends SendableChooser<V> {
    private final HashSet<V> optionSet = new HashSet<>();
    private final HashSet<V> ignoredSet = new HashSet<>();

    private V defaultOption;

    private boolean shouldIgnoreOption(final V object) {
        return ignoredSet.contains(object) || !object.hasCompetitionType(Constants.CURRENT_COMPETITION_TYPE);
    }

    /**
     * See {@link SendableChooser#setDefaultOption(String, Object)}
     */
    public void setDefaultOption(final String name, final V object) {
        super.setDefaultOption(name, object);
        this.defaultOption = object;
    }

    /**
     * See {@link AutoChooser#setDefaultOption(String, AutoOption)}
     */
    public void setDefaultAutoOption(final V object) {
        setDefaultOption(object.pathName(), object);
    }

    /**
     * See {@link SendableChooser#addOption(String, Object)}
     */
    public void addOption(final String name, final V object) {
        if (shouldIgnoreOption(object)) {
            ignoredSet.add(object);
            return;
        }

        super.addOption(name, object);
        optionSet.add(object);
    }

    /**
     * Adds an {@link AutoOption} object to the {@link SendableChooser} by using the name of the option
     * @param object the {@link AutoOption} or V
     * @see AutoOption
     * @see SendableChooser
     * @see AutoChooser#addOption(String, AutoOption)
     */
    public void addAutoOption(final V object) {
        addOption(object.pathName(), object);
    }

    public V getSelected() {
        final V selected = super.getSelected();
        return selected != null ? selected : defaultOption;
    }

    public void addOptionsIfNotPresent(
            final Function<V, String> nameFunction, final Function<I, V> objectFunction, final List<I> inputs
    ) {
        for (final I input : inputs) {
            final V computedObject = objectFunction.apply(input);
            if (!optionSet.contains(computedObject)) {
                addOption(nameFunction.apply(computedObject), computedObject);
            }
        }
    }
}
