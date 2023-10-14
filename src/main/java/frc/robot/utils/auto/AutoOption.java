package frc.robot.utils.auto;

import frc.robot.constants.Constants;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public record AutoOption(
        String name,
        Map<String, TitanTrajectory.Constraints> pathNameConstraintsMap,
        HashSet<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes =
            List.of(Constants.CompetitionType.TESTING);

    public AutoOption(
            final String name,
            final Map<String, TitanTrajectory.Constraints> pathNameConstraintsMap
    ) {
        this(
                name,
                pathNameConstraintsMap,
                new HashSet<>(defaultCompetitionTypes)
        );
    }

    public AutoOption(final String name, final List<String> pathNames) {
        this(
                name,
                pathNames.stream().collect(Collectors.toUnmodifiableMap(
                    pathName -> pathName,
                    pathName -> TitanTrajectory.Constraints.getDefault()
                ))
        );
    }

    public AutoOption(
            final String name,
            final Map<String, TitanTrajectory.Constraints> pathNameConstraintsMap,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                pathNameConstraintsMap,
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
                pathNames.stream().collect(Collectors.toUnmodifiableMap(
                        pathName -> pathName,
                        pathName -> TitanTrajectory.Constraints.getDefault()
                )),
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
        this(
                pathName,
                Map.of(pathName, new TitanTrajectory.Constraints(maxVelocity, maxAcceleration)),
                competitionTypes
        );
    }

    public AutoOption(final String pathName, final Constants.CompetitionType... competitionTypes) {
        this(pathName, List.of(pathName), competitionTypes);
    }

    public String getDescriptiveName() {
        return pathNameConstraintsMap.isEmpty() ? String.format("%s_EmptyPath@%s", name, this) : name;
    }

    public boolean equals(final Object other) {
        if (other == this) {
            return true;
        } else if (other instanceof AutoOption) {
            // since path names should be unique, just compare their names here
            return this.pathNameConstraintsMap.keySet().equals(((AutoOption) other).pathNameConstraintsMap.keySet());
        }

        return false;
    }

    public int hashCode() {
        return this.pathNameConstraintsMap.hashCode();
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }
}
