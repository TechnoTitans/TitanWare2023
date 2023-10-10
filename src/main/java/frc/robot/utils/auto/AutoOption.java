package frc.robot.utils.auto;

import frc.robot.constants.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Stream;

public record AutoOption(
        String name,
        List<String> pathNames,
        double maxVelocity,
        double maxAcceleration,
        HashSet<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes =
            List.of(Constants.CompetitionType.TESTING);

    public AutoOption(
            final String name,
            final List<String> pathNames,
            final double maxVelocity,
            final double maxAcceleration
    ) {
        this(
                name,
                pathNames,
                maxVelocity,
                maxAcceleration,
                new HashSet<>(defaultCompetitionTypes)
        );
    }

    public AutoOption(final String name, final List<String> pathNames) {
        this(name, pathNames, Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION);
    }

    public AutoOption(
            final String name,
            final List<String> pathNames,
            final double maxVelocity,
            final double maxAcceleration,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                pathNames,
                maxVelocity,
                maxAcceleration,
                new HashSet<>(Stream.concat(defaultCompetitionTypes.stream(), Arrays.stream(competitionTypes)).toList())
        );
    }

    public AutoOption(
            final String name,
            final List<String> pathNames,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                pathNames,
                Constants.Swerve.TRAJECTORY_MAX_SPEED,
                Constants.Swerve.TRAJECTORY_MAX_ACCELERATION,
                competitionTypes
        );
    }

    public AutoOption(final String pathName) {
        this(pathName, List.of(pathName));
    }

    public AutoOption(final String pathName,
                      final double maxVelocity,
                      final double maxAcceleration,
                      final Constants.CompetitionType... competitionTypes
    ) {
        this(pathName, List.of(pathName), maxVelocity, maxAcceleration, competitionTypes);
    }

    public AutoOption(final String pathName, final Constants.CompetitionType... competitionTypes) {
        this(pathName, List.of(pathName), competitionTypes);
    }

    public String getDescriptiveName() {
        return pathNames.isEmpty() ? String.format("%s_EmptyPath@%s", name, this) : name;
    }

    public boolean equals(final Object other) {
        if (other == this) {
            return true;
        } else if (other instanceof AutoOption) {
            // since path names should be unique, just compare their names here
            return this.pathNames.equals(((AutoOption) other).pathNames);
        }

        return false;
    }

    public int hashCode() {
        return this.pathNames.hashCode();
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }
}
