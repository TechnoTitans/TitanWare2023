package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class SwerveTest {
    private static final double EPSILON = 1E-7;

    @Mock
    private Gyro gyro;
    @Mock
    private PhotonVision photonVision;

    @Mock
    private SwerveModule frontLeft;
    @Mock
    private SwerveModule frontRight;
    @Mock
    private SwerveModule backLeft;
    @Mock
    private SwerveModule backRight;

    private Swerve swerve;

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            Constants.Swerve.FL_OFFSET,
            Constants.Swerve.FR_OFFSET,
            Constants.Swerve.BL_OFFSET,
            Constants.Swerve.BR_OFFSET
    );

    @BeforeAll
    static void beforeAll() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        if (swerve == null) {
            swerve = new Swerve(gyro, swerveDriveKinematics, frontLeft, frontRight, backLeft, backRight);
        }
    }

    @Test
    void periodic() {
        when(frontLeft.getState()).thenReturn(new SwerveModuleState());
        when(frontRight.getState()).thenReturn(new SwerveModuleState());
        when(backLeft.getState()).thenReturn(new SwerveModuleState());
        when(backRight.getState()).thenReturn(new SwerveModuleState());

        when(frontLeft.getLastDesiredState()).thenReturn(new SwerveModuleState());
        when(frontRight.getLastDesiredState()).thenReturn(new SwerveModuleState());
        when(backLeft.getLastDesiredState()).thenReturn(new SwerveModuleState());
        when(backRight.getLastDesiredState()).thenReturn(new SwerveModuleState());

        when(gyro.getYawRotation2d()).thenReturn(Rotation2d.fromDegrees(0));

        swerve.periodic();

        verify(gyro).periodic();
        verify(frontLeft).periodic();
        verify(frontRight).periodic();
        verify(backLeft).periodic();
        verify(backRight).periodic();
    }

    @Test
    void getGyro() {
        assertEquals(gyro, swerve.getGyro());
    }

    @ParameterizedTest
    @MethodSource("angleArgsProvider")
    void getPitch(final double angleDeg) {
        when(gyro.getPitch()).thenReturn(angleDeg);
        when(gyro.getPitchRotation2d()).thenCallRealMethod();

        assertEquals(Rotation2d.fromDegrees(angleDeg), swerve.getPitch());
        verify(gyro).getPitch();
    }

    @ParameterizedTest
    @MethodSource("angleArgsProvider")
    void getRoll(final double angleDeg) {
        when(gyro.getRoll()).thenReturn(angleDeg);
        when(gyro.getRollRotation2d()).thenCallRealMethod();

        assertEquals(Rotation2d.fromDegrees(angleDeg), swerve.getRoll());
        verify(gyro).getRoll();
    }

    @ParameterizedTest
    @MethodSource("angleArgsProvider")
    void getYaw(final double angleDeg) {
        when(gyro.getYaw()).thenReturn(angleDeg);
        when(gyro.getYawRotation2d()).thenCallRealMethod();

        assertEquals(Rotation2d.fromDegrees(angleDeg), swerve.getYaw());
        verify(gyro).getYaw();
    }

    @ParameterizedTest
    @MethodSource("angleArgsProvider")
    void setAngle(final double angleDeg) {
        swerve.setAngle(Rotation2d.fromDegrees(angleDeg));
        verify(gyro).setAngle(Rotation2d.fromDegrees(angleDeg));
    }

    static Stream<Double> angleArgsProvider() {
        return Stream.of(
                0d,
                180d,
                720d,
                1683d,
                -1771.2974,
                4188.1261,
                -99.99,
                -44.51
        );
    }

    @Test
    void zeroRotation() {
        doNothing().when(photonVision).resetPosition(any(), any());
        when(photonVision.getEstimatedPosition()).thenReturn(new Pose2d());
        doNothing().when(gyro).zeroRotation();

        swerve.zeroRotation(photonVision);
        verify(gyro).zeroRotation();
        verify(photonVision).resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
    }

    @Test
    void getRobotRelativeSpeeds() {
        when(frontLeft.getState()).thenReturn(new SwerveModuleState());
        when(frontRight.getState()).thenReturn(new SwerveModuleState());
        when(backLeft.getState()).thenReturn(new SwerveModuleState());
        when(backRight.getState()).thenReturn(new SwerveModuleState());

        final ChassisSpeeds chassisSpeeds = swerve.getRobotRelativeSpeeds();
        final ChassisSpeeds zero = new ChassisSpeeds();

        assertEquals(zero.vxMetersPerSecond, chassisSpeeds.vxMetersPerSecond, EPSILON);
        assertEquals(zero.vyMetersPerSecond, chassisSpeeds.vyMetersPerSecond, EPSILON);
        assertEquals(zero.omegaRadiansPerSecond, chassisSpeeds.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    void getFieldRelativeSpeeds() {
        when(frontLeft.getState()).thenReturn(new SwerveModuleState());
        when(frontRight.getState()).thenReturn(new SwerveModuleState());
        when(backLeft.getState()).thenReturn(new SwerveModuleState());
        when(backRight.getState()).thenReturn(new SwerveModuleState());

        when(gyro.getYawRotation2d()).thenReturn(Rotation2d.fromDegrees(0));

        final ChassisSpeeds chassisSpeeds = swerve.getFieldRelativeSpeeds();
        final ChassisSpeeds zero = new ChassisSpeeds();

        assertEquals(zero.vxMetersPerSecond, chassisSpeeds.vxMetersPerSecond, EPSILON);
        assertEquals(zero.vyMetersPerSecond, chassisSpeeds.vyMetersPerSecond, EPSILON);
        assertEquals(zero.omegaRadiansPerSecond, chassisSpeeds.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    void getModuleStates() {
        when(frontLeft.getState()).thenReturn(new SwerveModuleState());
        when(frontRight.getState()).thenReturn(new SwerveModuleState());
        when(backLeft.getState()).thenReturn(new SwerveModuleState());
        when(backRight.getState()).thenReturn(new SwerveModuleState());

        final SwerveModuleState[] swerveModuleStates = swerve.getModuleStates();
        assertEquals(4, swerveModuleStates.length);

        assertArrayEquals(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        }, swerveModuleStates);
    }

    @Test
    void getModuleLastDesiredStates() {
        when(frontLeft.getLastDesiredState()).thenReturn(new SwerveModuleState());
        when(frontRight.getLastDesiredState()).thenReturn(new SwerveModuleState());
        when(backLeft.getLastDesiredState()).thenReturn(new SwerveModuleState());
        when(backRight.getLastDesiredState()).thenReturn(new SwerveModuleState());

        final SwerveModuleState[] swerveModuleStates = swerve.getModuleLastDesiredStates();
        assertEquals(4, swerveModuleStates.length);

        assertArrayEquals(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        }, swerveModuleStates);
    }

    @Test
    void getModulePositions() {
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition());
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition());
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition());
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition());

        final SwerveModulePosition[] swerveModulePositions = swerve.getModulePositions();
        assertEquals(4, swerveModulePositions.length);

        assertArrayEquals(new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        }, swerveModulePositions);
    }

    // TODO: make this test better?
    @Test
    void driveWithStates() {
        swerve.drive(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        });

        verify(frontLeft).setDesiredState(any());
        verify(frontRight).setDesiredState(any());
        verify(backLeft).setDesiredState(any());
        verify(backRight).setDesiredState(any());
    }

    // TODO: make this test better?
    @Test
    void driveWithJoystick() {
        when(swerve.getYaw()).thenReturn(Rotation2d.fromDegrees(0));

        swerve.drive(0, 0, 0, true);
        verify(frontLeft).setDesiredState(any());
        verify(frontRight).setDesiredState(any());
        verify(backLeft).setDesiredState(any());
        verify(backRight).setDesiredState(any());
    }

    // TODO: make this test better?
    @Test
    void driveWithChassisSpeeds() {
        swerve.drive(new ChassisSpeeds());
        verify(frontLeft).setDesiredState(any());
        verify(frontRight).setDesiredState(any());
        verify(backLeft).setDesiredState(any());
        verify(backRight).setDesiredState(any());
    }

    @Test
    void stop() {
        swerve.stop();
        verify(frontLeft).stop();
        verify(frontRight).stop();
        verify(backLeft).stop();
        verify(backRight).stop();
    }

    // TODO: make this test better?
    @Test
    void faceDirection() {
        when(swerve.getYaw()).thenReturn(Rotation2d.fromDegrees(0));

        swerve.faceDirection(0, 0, Rotation2d.fromDegrees(0), true, 1);
        verify(frontLeft).setDesiredState(any());
        verify(frontRight).setDesiredState(any());
        verify(backLeft).setDesiredState(any());
        verify(backRight).setDesiredState(any());
    }

    // TODO: make this test better?
    @Test
    void rawSet() {
        swerve.rawSet(0, 0, 0, 0, 0, 0, 0, 0);
        verify(frontLeft).setDesiredState(any());
        verify(frontRight).setDesiredState(any());
        verify(backLeft).setDesiredState(any());
        verify(backRight).setDesiredState(any());
    }

    @Test
    void zero() {
        swerve.zero();
        verify(frontLeft).setDesiredState(any());
        verify(frontRight).setDesiredState(any());
        verify(backLeft).setDesiredState(any());
        verify(backRight).setDesiredState(any());
    }

    // TODO: make this test better?
    @Test
    void wheelX() {
        swerve.wheelX();
        verify(frontLeft).setDesiredState(any());
        verify(frontRight).setDesiredState(any());
        verify(backLeft).setDesiredState(any());
        verify(backRight).setDesiredState(any());
    }

    @Test
    void setNeutralMode() {
        final NeutralModeValue brake = NeutralModeValue.Brake;
        final NeutralModeValue coast = NeutralModeValue.Coast;

        if (Robot.isSimulation() && Constants.CTRE.DISABLE_NEUTRAL_MODE_IN_SIM) {
            verifyNoMoreInteractions(frontLeft, frontRight, backLeft, backRight);
            return;
        }

        swerve.setNeutralMode(brake);
        verify(frontLeft).setNeutralMode(brake);
        verify(frontRight).setNeutralMode(brake);
        verify(backLeft).setNeutralMode(brake);
        verify(backRight).setNeutralMode(brake);

        swerve.setNeutralMode(coast);
        verify(frontLeft).setNeutralMode(coast);
        verify(frontRight).setNeutralMode(coast);
        verify(backLeft).setNeutralMode(coast);
        verify(backRight).setNeutralMode(coast);
    }
}