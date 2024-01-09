package frc.robot.subsystems.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;

import java.util.List;

@SuppressWarnings("unused")
public class OrchestrateAll extends SubsystemBase {
    private final Orchestra orchestra;

    public OrchestrateAll(
            final String musicFile,
            final int nTracks,
            final HardwareConstants.SwerveModuleConstants frontLeftModule,
            final HardwareConstants.SwerveModuleConstants frontRightModule,
            final HardwareConstants.SwerveModuleConstants backLeftModule,
            final HardwareConstants.SwerveModuleConstants backRightModule,
            final HardwareConstants.ElevatorConstants elevatorConstants
    ) {
        final TalonFX frontLeftDrive = new TalonFX(frontLeftModule.driveMotorId(), frontLeftModule.moduleCANBus());
        final TalonFX frontLeftTurn = new TalonFX(frontLeftModule.turnMotorId(), frontLeftModule.moduleCANBus());
        final TalonFX frontRightDrive = new TalonFX(frontRightModule.driveMotorId(), frontRightModule.moduleCANBus());
        final TalonFX frontRightTurn = new TalonFX(frontRightModule.turnMotorId(), frontRightModule.moduleCANBus());
        final TalonFX backLeftDrive = new TalonFX(backLeftModule.driveMotorId(), backLeftModule.moduleCANBus());
        final TalonFX backLeftTurn = new TalonFX(backLeftModule.turnMotorId(), backLeftModule.moduleCANBus());
        final TalonFX backRightDrive = new TalonFX(backRightModule.driveMotorId(), backRightModule.moduleCANBus());
        final TalonFX backRightTurn = new TalonFX(backRightModule.turnMotorId(), backRightModule.moduleCANBus());

        final TalonFX verticalElevatorMain = new TalonFX(
                elevatorConstants.verticalMainMotorId(),
                elevatorConstants.verticalElevatorCANBus()
        );
        final TalonFX verticalElevatorFollower = new TalonFX(
                elevatorConstants.verticalFollowerMotorId(),
                elevatorConstants.verticalElevatorCANBus()
        );
        final TalonFX horizontalElevatorMotor = new TalonFX(
                elevatorConstants.horizontalMotorId(),
                elevatorConstants.horizontalElevatorCANBus()
        );

        final List<ParentDevice> devices = List.of(
                frontLeftDrive, frontLeftTurn, frontRightDrive, frontRightTurn,
                backLeftDrive, backLeftTurn, backRightDrive, backRightTurn,
                verticalElevatorMain, verticalElevatorFollower,
                horizontalElevatorMotor
        );
        this.orchestra = new Orchestra(devices, musicFile);

        final int devicesPerTrack = (int) Math.floor((double)devices.size() / nTracks);

        // Thank you Dr. Reddy for sponsoring this segment of the code
        for (int trackN = 0; trackN < nTracks; trackN++) {
            for (int deviceN = 0; deviceN < devicesPerTrack; deviceN++) {
                final ParentDevice device = devices.get((trackN * devicesPerTrack) + deviceN);
                Phoenix6Utils.reportIfNotOk(device, orchestra.addInstrument(device));
            }
        }

        final int leftoverDevices = devices.size() % nTracks;
        for (int i = devices.size() - leftoverDevices - 1; i < devices.size(); i++) {
            final ParentDevice device = devices.get(i);
            Phoenix6Utils.reportIfNotOk(device, orchestra.addInstrument(device));
        }

        this.orchestra.loadMusic(musicFile);
    }

    public Command playCommand() {
        return runOnce(orchestra::play).ignoringDisable(true);
    }

    public Command pauseCommand() {
        return runOnce(orchestra::pause).ignoringDisable(true);
    }

    public Command stopCommand() {
        return runOnce(orchestra::stop).ignoringDisable(true);
    }
}
