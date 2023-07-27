package frc.robot.utils.auto;

import frc.robot.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Stream;

public record AutoOption(
        String pathName, double maxVelocity, double maxAcceleration, HashSet<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes = List.of(Constants.CompetitionType.TESTING);

    public AutoOption(final String pathName, final double maxVelocity, final double maxAcceleration) {
        this(
                pathName,
                maxVelocity,
                maxAcceleration,
                new HashSet<>(defaultCompetitionTypes)
        );
    }

    public AutoOption(final String pathName) {
        this(pathName, Constants.Swerve.TRAJECTORY_MAX_SPEED, Constants.Swerve.TRAJECTORY_MAX_ACCELERATION);
    }

    public AutoOption(
            final String pathName,
            final double maxVelocity,
            final double maxAcceleration,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                pathName,
                maxVelocity,
                maxAcceleration,
                new HashSet<>(Stream.concat(defaultCompetitionTypes.stream(), Arrays.stream(competitionTypes)).toList())
        );
    }

    public AutoOption(final String pathName, final Constants.CompetitionType... competitionTypes) {
        this(
                pathName,
                Constants.Swerve.TRAJECTORY_MAX_SPEED,
                Constants.Swerve.TRAJECTORY_MAX_ACCELERATION,
                competitionTypes
        );
    }

    public boolean equals(final Object other) {
        if (other == this) {
            return true;
        } else if (other instanceof AutoOption) {
            // since path names should be unique, just compare their names here
            return this.pathName.equals(((AutoOption) other).pathName);
        }

        return false;
    }

    public int hashCode() {
        return this.pathName.hashCode();
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }
}
