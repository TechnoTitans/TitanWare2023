package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.wrappers.sensors.vision.PhotonVision;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.Mock;
import org.mockito.Spy;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.Arrays;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class SwerveTest {
    private static final double EPSILON = 1E-7;

    @Mock
    private Gyro gyro;
    @Mock
    private PhotonVision<?> photonVision;

    @Spy
    private final SwerveModule frontLeft = new SwerveModule(new SwerveModuleIO() {}, "FrontLeft");
    @Spy
    private final SwerveModule frontRight = new SwerveModule(new SwerveModuleIO() {}, "FrontRight");
    @Spy
    private final SwerveModule backLeft = new SwerveModule(new SwerveModuleIO() {}, "BackLeft");
    @Spy
    private final SwerveModule backRight = new SwerveModule(new SwerveModuleIO() {}, "BackRight");

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
            when(gyro.getYawRotation2d()).thenReturn(Rotation2d.fromDegrees(0));

            swerve = new Swerve(gyro, swerveDriveKinematics, frontLeft, frontRight, backLeft, backRight);
        }
    }

    @Test
    void periodic() {
        when(gyro.getYawRotation2d()).thenReturn(Rotation2d.fromDegrees(0));
        when(gyro.getPitchRotation2d()).thenReturn(Rotation2d.fromDegrees(0));
        when(gyro.getRollRotation2d()).thenReturn(Rotation2d.fromDegrees(0));

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
        final ChassisSpeeds chassisSpeeds = swerve.getRobotRelativeSpeeds();
        final ChassisSpeeds zero = new ChassisSpeeds();

        assertEquals(zero.vxMetersPerSecond, chassisSpeeds.vxMetersPerSecond, EPSILON);
        assertEquals(zero.vyMetersPerSecond, chassisSpeeds.vyMetersPerSecond, EPSILON);
        assertEquals(zero.omegaRadiansPerSecond, chassisSpeeds.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    void getFieldRelativeSpeeds() {
        when(gyro.getYawRotation2d()).thenReturn(Rotation2d.fromDegrees(0));

        final ChassisSpeeds chassisSpeeds = swerve.getFieldRelativeSpeeds();
        final ChassisSpeeds zero = new ChassisSpeeds();

        assertEquals(zero.vxMetersPerSecond, chassisSpeeds.vxMetersPerSecond, EPSILON);
        assertEquals(zero.vyMetersPerSecond, chassisSpeeds.vyMetersPerSecond, EPSILON);
        assertEquals(zero.omegaRadiansPerSecond, chassisSpeeds.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    void getModuleStates() {
        final SwerveModuleState[] swerveModuleStates = swerve.getModuleStates();

        assertEquals(4, swerveModuleStates.length);
        assertArrayEquals(new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        }, swerveModuleStates);
    }

    static Stream<Arguments> provideSwerveModuleStates() {
        return Stream.of(
                Arguments.of(
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0))
                ),
                Arguments.of(
                        new SwerveModuleState(0.824, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(1.524, Rotation2d.fromDegrees(-188.5)),
                        new SwerveModuleState(-0.252, Rotation2d.fromRadians(Math.PI)),
                        new SwerveModuleState(-1.1145, Rotation2d.fromRotations(-0.66))
                )
        );
    }

    private static boolean optimizedStateEquals(
            final SwerveModuleState measured,
            final SwerveModuleState desired,
            final SwerveModuleState optimized
    ) {
        if (desired.equals(optimized)) {
            return true;
        }

        final SwerveModuleState desiredOptimized = SwerveModuleState.optimize(desired, measured.angle);
        if (Constants.Swerve.USE_SWERVE_COSINE_SCALING) {
            SwerveModule.scaleWithErrorCosine(desiredOptimized, measured.angle);
        }

        return desiredOptimized.equals(optimized);
    }

    private static boolean optimizedStatesEquals(
            final SwerveModuleState[] measured,
            final SwerveModuleState[] desired,
            final SwerveModuleState[] optimized
    ) {
        if (desired.length != optimized.length) {
            return false;
        } else if (Arrays.equals(desired, optimized)) {
            return true;
        }

        for (int i = 0; i < desired.length; i++) {
            if (!optimizedStateEquals(measured[i], desired[i], optimized[i])) {
                return false;
            }
        }

        return true;
    }

    @ParameterizedTest
    @MethodSource("provideSwerveModuleStates")
    void getModuleLastDesiredStates(
            final SwerveModuleState frontLeftState,
            final SwerveModuleState frontRightState,
            final SwerveModuleState backLeftState,
            final SwerveModuleState backRightState
    ) {
        frontLeft.setDesiredState(frontLeftState);
        frontRight.setDesiredState(frontRightState);
        backLeft.setDesiredState(backLeftState);
        backRight.setDesiredState(backRightState);

        final SwerveModuleState[] swerveModuleStates = swerve.getModuleLastDesiredStates();

        assertEquals(4, swerveModuleStates.length);
        assertTrue(optimizedStatesEquals(
                swerve.getModuleStates(),
                new SwerveModuleState[] {
                    frontLeftState,
                    frontRightState,
                    backLeftState,
                    backRightState
                },
                swerveModuleStates
        ));
    }

    @Test
    void getModulePositions() {
        final SwerveModulePosition[] swerveModulePositions = swerve.getModulePositions();
        assertEquals(4, swerveModulePositions.length);

        assertArrayEquals(new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        }, swerveModulePositions);
    }

    @ParameterizedTest
    @MethodSource("provideSwerveModuleStates")
    void driveWithStates(
            final SwerveModuleState frontLeftState,
            final SwerveModuleState frontRightState,
            final SwerveModuleState backLeftState,
            final SwerveModuleState backRightState
    ) {
        final SwerveModuleState[] desiredStates = new SwerveModuleState[] {
                frontLeftState,
                frontRightState,
                backLeftState,
                backRightState
        };

        swerve.drive(desiredStates);

        assertTrue(optimizedStatesEquals(
                swerve.getModuleStates(),
                desiredStates,
                swerve.getModuleLastDesiredStates()
        ));
    }

    @Test
    void driveWithJoystick() {
        when(swerve.getYaw()).thenReturn(Rotation2d.fromDegrees(0));

        swerve.drive(0, 0, 0, true);
        assertEquals(new SwerveModuleState(), frontLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), frontRight.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backRight.getLastDesiredState());
    }

    @Test
    void driveWithChassisSpeeds() {
        swerve.drive(new ChassisSpeeds());

        assertEquals(new SwerveModuleState(), frontLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), frontRight.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backRight.getLastDesiredState());
    }

    @Test
    void stop() {
        swerve.stop();

        assertEquals(
                new SwerveModuleState(0, frontLeft.getAngle()),
                frontLeft.getLastDesiredState()
        );
        assertEquals(
                new SwerveModuleState(0, frontRight.getAngle()),
                frontRight.getLastDesiredState()
        );
        assertEquals(
                new SwerveModuleState(0, backLeft.getAngle()),
                backLeft.getLastDesiredState()
        );
        assertEquals(
                new SwerveModuleState(0, backRight.getAngle()),
                backRight.getLastDesiredState()
        );
    }

    @Test
    void faceDirection() {
        when(swerve.getYaw()).thenReturn(Rotation2d.fromDegrees(0));

        swerve.faceDirection(0, 0, Rotation2d.fromDegrees(0), true, 1);
        assertEquals(new SwerveModuleState(), frontLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), frontRight.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backRight.getLastDesiredState());
    }

    @ParameterizedTest
    @MethodSource("provideRawSet")
    void rawSet(
            final double s1,
            final double s2,
            final double s3,
            final double s4,
            final double a1,
            final double a2,
            final double a3,
            final double a4
    ) {
        swerve.rawSet(s1, s2, s3, s4, a1, a2, a3, a4);

        assertTrue(optimizedStatesEquals(
                swerve.getModuleStates(),
                new SwerveModuleState[] {
                        new SwerveModuleState(s1, Rotation2d.fromDegrees(a1)),
                        new SwerveModuleState(s2, Rotation2d.fromDegrees(a2)),
                        new SwerveModuleState(s3, Rotation2d.fromDegrees(a3)),
                        new SwerveModuleState(s4, Rotation2d.fromDegrees(a4))
                },
                swerve.getModuleLastDesiredStates()
        ));
    }

    static Stream<Arguments> provideRawSet() {
        return Stream.of(
                Arguments.of(0, 0, 0, 0, 0, 0, 0, 0),
                Arguments.of(1, -1, -0.0825, 0.451, -45, 0.00245, -30, 180.22),
                Arguments.of(-3.215, 1.1568, 0.982, 0, 0, 1.25, 60, -45),
                Arguments.of(-0.09, -2.44, 1.11, 0.01, -1, 0.251, -90, -180)
        );
    }

    @Test
    void zero() {
        swerve.zero();

        assertEquals(new SwerveModuleState(), frontLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), frontRight.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backLeft.getLastDesiredState());
        assertEquals(new SwerveModuleState(), backRight.getLastDesiredState());
    }

    @Test
    void wheelX() {
        swerve.wheelX();

        assertEquals(
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                frontLeft.getLastDesiredState()
        );
        assertEquals(
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                frontRight.getLastDesiredState()
        );
        assertEquals(
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                backLeft.getLastDesiredState()
        );
        assertEquals(
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                backRight.getLastDesiredState()
        );
    }

    @ParameterizedTest
    @EnumSource(NeutralModeValue.class)
    void setNeutralMode(final NeutralModeValue neutralMode) {
        swerve.setNeutralMode(neutralMode);

        verify(frontLeft).setNeutralMode(neutralMode);
        verify(frontRight).setNeutralMode(neutralMode);
        verify(backLeft).setNeutralMode(neutralMode);
        verify(backRight).setNeutralMode(neutralMode);
    }
}