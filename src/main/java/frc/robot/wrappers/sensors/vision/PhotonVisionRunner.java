package frc.robot.wrappers.sensors.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface PhotonVisionRunner {
    default void periodic() {};
    default void resetRobotPose(final Pose3d pose3d) {};
}
