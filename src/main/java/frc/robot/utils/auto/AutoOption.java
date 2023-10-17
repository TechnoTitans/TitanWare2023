package frc.robot.utils.auto;

import frc.robot.constants.Constants;

import java.util.*;
import java.util.stream.Stream;

public record AutoOption(
        String name,
        List<PathNameWithConstraints> pathNameWithConstraintsList,
        HashSet<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes =
            List.of(Constants.CompetitionType.TESTING);

    public record PathNameWithConstraints(String name, TitanTrajectory.Constraints constraints) {
        public PathNameWithConstraints(final String name) {
            this(name, TitanTrajectory.Constraints.getDefault());
        }
    }

    public AutoOption(
            final String name,
            final List<PathNameWithConstraints> pathNameWithConstraints
    ) {
        this(
                name,
                pathNameWithConstraints,
                new HashSet<>(defaultCompetitionTypes)
        );
    }

    public AutoOption(
            final String name,
            final List<PathNameWithConstraints> pathNameWithConstraints,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                pathNameWithConstraints,
                new HashSet<>(Stream.concat(defaultCompetitionTypes.stream(), Arrays.stream(competitionTypes)).toList())
        );
    }

    public AutoOption(final String pathName) {
        this(pathName, List.of(new PathNameWithConstraints(pathName)));
    }

    public AutoOption(final String pathName,
                      final double maxVelocity,
                      final double maxAcceleration,
                      final Constants.CompetitionType... competitionTypes
    ) {
        this(
                pathName,
                List.of(new PathNameWithConstraints(
                        pathName, new TitanTrajectory.Constraints(maxVelocity, maxAcceleration)
                )),
                competitionTypes
        );
    }

    public AutoOption(final String pathName, final Constants.CompetitionType... competitionTypes) {
        this(pathName, List.of(new PathNameWithConstraints(pathName)), competitionTypes);
    }

    public String getDescriptiveName() {
        return pathNameWithConstraintsList.isEmpty() ? String.format("%s_EmptyPath@%s", name, this) : name;
    }

    public boolean equals(final Object other) {
        if (other == this) {
            return true;
        } else if (other instanceof AutoOption) {
            // since path names should be unique, just compare their names here
            return this.pathNameWithConstraintsList.equals(((AutoOption) other).pathNameWithConstraintsList);
        }

        return false;
    }

    public int hashCode() {
        return this.pathNameWithConstraintsList.hashCode();
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }
}
