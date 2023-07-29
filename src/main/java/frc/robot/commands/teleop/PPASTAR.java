package frc.robot.commands.teleop;

import com.ctre.phoenix6.sim.ChassisReference;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.pathfinding.Edge;
import frc.robot.commands.pathfinding.Node;
import frc.robot.commands.pathfinding.Obstacle;
import frc.robot.commands.pathfinding.VisGraph;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.wrappers.sensors.vision.PhotonVision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PPAStar extends CommandBase {
    private final Swerve swerve;
    private final PhotonVision photonVision;
    private PPSwerveControllerCommand pathDrivingCommand;
    private final Node finalPosition;
    private Node startPoint;
    private final List<Obstacle> obstacles;
    private final VisGraph AStarMap;

    public PPAStar(
            final Swerve swerve,
            final PhotonVision photonVision,
            final Node finalPosition,
            final List<Obstacle> obstacles,
            final VisGraph AStarMap
    ) {
        this.swerve = swerve;
        this.photonVision = photonVision;
        this.obstacles = obstacles;
        this.finalPosition = finalPosition;
        this.AStarMap = AStarMap;
        this.startPoint = new Node(photonVision);

        addRequirements(swerve);
    }

    // ----------------------------------------------------------------------------
    // Per-schedule setup code.
    @Override
    public void initialize() {
        startPoint = new Node(photonVision);
        final PathPlannerTrajectory trajectory;
        List<Node> fullPath = new ArrayList<>();

        AStarMap.addNode(startPoint);
        if (AStarMap.addEdge(new Edge(startPoint, finalPosition), obstacles)) {
            fullPath.add(0, startPoint);
            fullPath.add(1, finalPosition);
        } else {
            for (int i = 0; i < AStarMap.getNodeSize(); i++) {
                final Node endNode = AStarMap.getNode(i);
                AStarMap.addEdge(new Edge(startPoint, endNode), obstacles);
            }
            fullPath = AStarMap.findPath(startPoint, finalPosition);
        }

        if(fullPath == null){
            return;
        }
        final Pose2d currentPose = photonVision.getEstimatedPosition();
        final ChassisSpeeds robotRelativeSpeeds = swerve.getRobotRelativeSpeeds();
        final double startingSpeed = Math.hypot(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);


        final Rotation2d heading;
        if (startingSpeed>0.05){
            heading = new Rotation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
        } else {
            heading = new Rotation2d(fullPath.get(1).getX()-startPoint.getX(),fullPath.get(1).getY()-startPoint.getY());
        }

        // Depending on if internal points are present, make a new array of the other
        // points in the path.
        final PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
        final List<PathPoint> fullPathPoints1 = new ArrayList<>();

        for (int i = 0; i < fullPath.size(); i++) {
            if (i == 0) {
                fullPathPoints[i] = new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), heading,
                        currentPose.getRotation(), startingSpeed);
            } else if (i + 1 == fullPath.size()) {
                fullPathPoints[i] = new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
                        new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
                        finalPosition.getHolRot());
            } else {
                fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
                        new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
                        finalPosition.getHolRot());
            }

        }

        // Declare an array to hold PathPoint objects made from all other points
        // specified in constructor.

//        trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
//        photonVision.addTrajectory(trajectory);
//        pathDrivingCommand = DrivetrainSubsystem.followTrajectory(swerve, photonVision, trajectory);
//        pathDrivingCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return (pathDrivingCommand == null || !pathDrivingCommand.isScheduled());
    }

    @Override
    public void end(boolean interrupted) {
        pathDrivingCommand.cancel();
        swerve.stop();
    }

    public static double angleAtPercent(double start, double end, double percent) {
        double angleDiff = end - start;
        if (angleDiff > 180) {
            angleDiff -= 360;
        } else if (angleDiff < -180) {
            angleDiff += 360;
        }
        double angle = start + (angleDiff * percent);
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}
