package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.TankDrive;

import java.io.IOException;
import java.nio.file.Path;

@SuppressWarnings("unused")
public class TrajectoryManager {

    public TrajectoryManager() {}

    public Trajectory createTrajectory(String path) {
        Trajectory trajectory;
        try {
            Path p = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(p);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
            return null;
        }
    }

    public RamseteCommand createRamseteCommand(Trajectory trajectory, TankDrive driveTrain) {
        return new RamseteCommand(
                trajectory,
                driveTrain::getPose,
                new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA),
                Constants.kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
                new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
                driveTrain::tankDriveVolts,
                driveTrain
        );
    }
    
}
