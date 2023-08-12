package frc.robot.commands.autoalign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.TrajectoryManager;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.alignment.AlignmentZone;
import frc.robot.utils.alignment.GridNode;
import frc.robot.utils.alignment.NTGridNode;
import frc.robot.utils.alignment.SubstationNode;
import frc.robot.utils.auto.TitanTrajectory;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.ElevatorClawCommand;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.EnumSet;

public class AutoAlignmentV3 extends CommandBase {
    protected static final String logKey = "AutoAlign/";

    private static NTGridNode SelectedNTGridNode = NTGridNode.UNKNOWN;
    private static final NetworkTable NTGameNodeTable = NetworkTableInstance.getDefault().getTable("GameNodeSelector");

    private static final IntegerTopic NTGridNodeIdTopic = NTGameNodeTable.getIntegerTopic("GridNodeId");
    private static final IntegerSubscriber NTGridNodeSubscriber = NTGridNodeIdTopic.subscribe(SelectedNTGridNode.getNtID());
    private static final IntegerPublisher NTGridNodePublisher = NTGridNodeIdTopic.publish();

    private static final StringTopic NTGridNodeNameTopic = NTGameNodeTable.getStringTopic("GridNodeName");
    private static final StringPublisher NTSelectedGridNodePublisher = NTGridNodeNameTopic.publish();


    static {
        NTGridNodePublisher.setDefault(SelectedNTGridNode.getNtID());
        NTSelectedGridNodePublisher.setDefault(SelectedNTGridNode.toString());

        NetworkTableInstance.getDefault().addListener(
                NTGridNodeSubscriber,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                (event) -> {
                    SelectedNTGridNode = NTGridNode.fromNtID(NTGridNodeSubscriber.get());
                    NTSelectedGridNodePublisher.set(SelectedNTGridNode.name());
                }
        );
    }

    private final Swerve swerve;
    private final Elevator elevator;
    private final Claw claw;
    private final CommandXboxController driverController;
    private final PhotonVision photonVision;
    private final TrajectoryManager trajectoryManager;

    private AlignmentZone.TrajectoryAlignmentSide desiredTrajectoryAlignmentSide;
    private SequentialCommandGroup commandGroup;

    public AutoAlignmentV3(
            final Swerve swerve,
            final Elevator elevator,
            final Claw claw,
            final CommandXboxController driverController,
            final PhotonVision photonVision,
            final TrajectoryManager trajectoryManager
    ) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.claw = claw;
        this.driverController = driverController;
        this.photonVision = photonVision;
        this.trajectoryManager = trajectoryManager;
    }

    public AutoAlignmentV3 withDesiredAlignmentSide(final AlignmentZone.TrajectoryAlignmentSide trajectoryAlignmentSide) {
        this.desiredTrajectoryAlignmentSide = trajectoryAlignmentSide;
        return this;
    }

    @Override
    public void initialize() {
        Logger.getInstance().recordOutput(logKey + "IsActive", true);

        if (desiredTrajectoryAlignmentSide == null) {
            cancel();
            return;
        }

        final Pose2d currentPose = photonVision.getEstimatedPosition();
        final AlignmentZone trajectoryAlignmentZone =
                AlignmentZone.getAlignmentZoneFromCurrentPose(currentPose);

        if (trajectoryAlignmentZone == null) {
            cancel();
            return;
        }

        Logger.getInstance().recordOutput(
                logKey + "TrajectoryAlignmentZone",
                trajectoryAlignmentZone.toString()
        );

        ElevatorClawCommand beforeElevatorClawCommand = null;
        ElevatorClawCommand afterElevatorClawCommand = null;
        AlignmentZone directAlignmentZone = null;
        AlignmentZone.GenericDesiredAlignmentPosition desiredAlignmentPosition = null;

        final boolean canAlign = switch (trajectoryAlignmentZone.getAlignmentZoneType()) {
            case TRAJECTORY_GRID, GRID -> {
                final long ntGridNodeId = NTGridNodeSubscriber.get();
                final GridNode gridNode = GridNode.getFromNT(SelectedNTGridNode).orElse(null);

                Logger.getInstance().recordOutput(
                        logKey + "NTGridNodeId", ntGridNodeId
                );
                Logger.getInstance().recordOutput(
                        logKey + "NTGridNode", SelectedNTGridNode.toString()
                );
                Logger.getInstance().recordOutput(
                        logKey + "GridNode", (gridNode != null) ? gridNode.toString() : "None"
                );

                if (gridNode == null) {
                    yield false;
                }

                afterElevatorClawCommand = gridNode.buildScoringSequence(elevator, claw);
                directAlignmentZone = gridNode.getAlignmentZone();
                desiredAlignmentPosition = gridNode.getAlignmentPosition();

                yield true;
            }
            case TRAJECTORY_SUBSTATION, SUBSTATION -> {
                final SubstationNode substationNode = SubstationNode
                        .getFromTrajectoryAlignmentSide(desiredTrajectoryAlignmentSide)
                        .orElse(null);

                if (substationNode == null) {
                    yield false;
                }

                beforeElevatorClawCommand = substationNode.buildIntakeSequence(elevator, claw);
                afterElevatorClawCommand = substationNode.buildRetractSequence(elevator, claw);
                directAlignmentZone = substationNode.getAlignmentZone();
                desiredAlignmentPosition = substationNode.getAlignmentPosition();

                yield true;
            }
        };

        if (!canAlign
                || directAlignmentZone == null
                || desiredAlignmentPosition == null
        ) {
            cancel();
            return;
        }

        this.commandGroup = new SequentialCommandGroup();

        final Pose2d targetPose = directAlignmentZone.getAlignmentPosition(desiredAlignmentPosition);
        final AlignmentZone.AlignmentZoneKind alignmentZoneKind = trajectoryAlignmentZone
                .getAlignmentZoneType()
                .getAlignmentZoneKind();

        if (alignmentZoneKind == AlignmentZone.AlignmentZoneKind.TRAJECTORY) {
            final ChassisSpeeds swerveChassisSpeeds = swerve.getFieldRelativeSpeeds();

            final Pose2d trajectoryTarget = trajectoryAlignmentZone.getTrajectoryTarget(desiredTrajectoryAlignmentSide);
            final TitanTrajectory.Constraints constraints = TitanTrajectory.Constraints.getDefault();

            final double approxDistance = trajectoryTarget.getTranslation().getDistance(targetPose.getTranslation());
            final double endVelocity = (constraints.maxAcceleration * approxDistance) / constraints.maxVelocity;

            final TitanTrajectory trajectory = new TitanTrajectory.Builder()
                    .withConstraints(TitanTrajectory.Constraints.getDefault())
                    .add(currentPose, swerveChassisSpeeds)
                    .add(trajectoryTarget)
                    .withEndVelocityOverride(endVelocity)
                    .build();

            Logger.getInstance().recordOutput(
                    logKey + "Trajectory",
                    LogUtils.LoggableTrajectory.fromTrajectory(trajectory)
            );

            commandGroup.addCommands(trajectoryManager.getCommand(trajectory));
        }

        //Score Sequence
        Logger.getInstance().recordOutput(
                logKey + "AlignmentZone", directAlignmentZone.toString()
        );
        Logger.getInstance().recordOutput(
                logKey + "AlignmentPosition", desiredAlignmentPosition.toString()
        );
        Logger.getInstance().recordOutput(
                logKey + "TargetPose", targetPose
        );

        switch (directAlignmentZone.getAlignmentZoneType()) {
            case GRID -> {
                final LineUpToGrid lineUpToGrid = new LineUpToGrid(swerve, photonVision, targetPose);
                if (afterElevatorClawCommand != null) {
                    commandGroup.addCommands(
                            lineUpToGrid,
                            afterElevatorClawCommand,
                            Commands.runOnce(() -> {
                                NTGridNodePublisher.set(NTGridNode.UNKNOWN.getNtID());
                            })
                    );
                }
            }
            case SUBSTATION -> {
                final Transform2d offsetBackwardsTransform = targetPose.minus(
                        new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(0))
                );

                final Pose2d offsetBackwardsPose = new Pose2d(
                        offsetBackwardsTransform.getTranslation(), offsetBackwardsTransform.getRotation()
                );

                final DriveToPose driveUpToSubstation = new DriveToPose(swerve, photonVision, offsetBackwardsPose);
                final DriveToPose driveForwardToIntakeSubstation = new DriveToPose(swerve, photonVision, targetPose);
                final DriveToPose driveBackFromSubstation = new DriveToPose(swerve, photonVision, offsetBackwardsPose);

                if (beforeElevatorClawCommand != null && afterElevatorClawCommand != null) {
                    commandGroup.addCommands(
                            driveUpToSubstation,
                            Commands.sequence(beforeElevatorClawCommand, driveForwardToIntakeSubstation),
                            Commands.waitSeconds(0.5),
                            Commands.parallel(driveBackFromSubstation, afterElevatorClawCommand)
                    );
                }
            }
        }

        commandGroup.addCommands(
                ControllerUtils.getRumbleForDurationCommand(
                        driverController.getHID(), GenericHID.RumbleType.kBothRumble, 0.75, 0.5
                )
        );
        commandGroup.schedule();
    }

    @Override
    public boolean isFinished() {
        return commandGroup.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (commandGroup != null) {
            commandGroup.cancel();
        }

        Logger.getInstance().recordOutput(logKey + "IsActive", false);
    }
}