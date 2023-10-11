package frc.robot.subsystems.drive;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimPhoenix6CANCoder;
import frc.robot.utils.sim.motors.CTREPhoenix6TalonFXSim;
import frc.robot.wrappers.control.Slot0Configs;

import java.util.*;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveModuleIOFalconSim implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CTREPhoenix6TalonFXSim driveSim;
    private final CTREPhoenix6TalonFXSim turnSim;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final InvertedValue driveInvertedValue;
    private final InvertedValue turnInvertedValue;
    private final TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonFXConfiguration = new TalonFXConfiguration();

    private final VelocityVoltage velocityVoltage;
    private final PositionVoltage positionVoltage;

    private final DeltaTime deltaTime;
    private static class StatusSignalQueue extends Thread {
        private static final int DEFAULT_CAPACITY = 8;
        private record StatusSignalMeasurement<T>(
                T value,
                StatusCode statusCode,
                Timestamp bestTimestamp,
                AllTimestamps allTimestamps
        ) {
            private static final Comparator<StatusSignalMeasurement<?>> comparator =
                    Comparator.comparingDouble(signalMeasurement -> signalMeasurement.bestTimestamp.getTime());

            public StatusSignalMeasurement(final StatusSignal<T> signal) {
                this(signal.getValue(), signal.getStatus(), signal.getTimestamp(), signal.getAllTimestamps());
            }

            public static StatusSignalMeasurement<Double> latencyCompensatedMeasurement(
                    final StatusSignal<Double> signal,
                    final StatusSignal<Double> deltaSignal
            ) {
                return new StatusSignalMeasurement<>(
                        BaseStatusSignal.getLatencyCompensatedValue(signal, deltaSignal),
                        signal.getStatus(),
                        signal.getTimestamp(),
                        signal.getAllTimestamps()
                );
            }
        }

        public record SignalMeasurementArrayQueue<T>(T[] values, double[] timestamps) {}

        private final ReentrantLock signalQueueLock;
        private final List<StatusSignal<?>> statusSignals;
        private final TalonFX driveMotor;
        private final CANcoder turnEncoder;

        private final List<StatusCode> statusCodeBadQueue;
        private final List<StatusSignalMeasurement<Double>> drivePositionSignalQueue;
        private final List<StatusSignalMeasurement<Double>> driveVelocitySignalQueue;
        private final List<StatusSignalMeasurement<Double>> turnPositionSignalQueue;
        private final List<StatusSignalMeasurement<Double>> turnVelocitySignalQueue;

        public StatusSignalQueue(final TalonFX driveMotor, final CANcoder turnEncoder) {
            super(String.format("StatusSignalQueue@%s.%s", driveMotor.getDeviceID(), turnEncoder.getDeviceID()));

            this.signalQueueLock = new ReentrantLock();
            this.statusSignals = List.of(
                    driveMotor.getPosition(), driveMotor.getVelocity(),
                    turnEncoder.getAbsolutePosition(), turnEncoder.getVelocity()
            );

            this.driveMotor = driveMotor;
            this.turnEncoder = turnEncoder;

            this.statusCodeBadQueue = new ArrayList<>(DEFAULT_CAPACITY);
            this.drivePositionSignalQueue = new ArrayList<>(DEFAULT_CAPACITY);
            this.driveVelocitySignalQueue = new ArrayList<>(DEFAULT_CAPACITY);
            this.turnPositionSignalQueue = new ArrayList<>(DEFAULT_CAPACITY);
            this.turnVelocitySignalQueue = new ArrayList<>(DEFAULT_CAPACITY);
        }

        @SuppressWarnings("InfiniteLoopStatement")
        @Override
        public void run() {
            final StatusSignal<?>[] statusSignalsArray = statusSignals.toArray(StatusSignal[]::new);
            while (true) {
                final StatusCode statusCode = BaseStatusSignal.waitForAll(0.1, statusSignalsArray);
                if (!statusCode.isOK()) {
                    signalQueueLock.lock();
                    try {
                        statusCodeBadQueue.add(statusCode);
                    } finally {
                        signalQueueLock.unlock();
                    }

                    continue;
                }

                signalQueueLock.lock();
                try {
                    final StatusSignal<Double> driveVelocitySignal = driveMotor.getVelocity();
                    drivePositionSignalQueue.add(StatusSignalMeasurement.latencyCompensatedMeasurement(
                            driveMotor.getPosition(),
                            driveVelocitySignal
                    ));
                    driveVelocitySignalQueue.add(new StatusSignalMeasurement<>(driveVelocitySignal));

                    final StatusSignal<Double> turnVelocitySignal = turnEncoder.getVelocity();
                    turnPositionSignalQueue.add(StatusSignalMeasurement.latencyCompensatedMeasurement(
                            turnEncoder.getAbsolutePosition(),
                            turnVelocitySignal
                    ));
                    turnVelocitySignalQueue.add(new StatusSignalMeasurement<>(turnVelocitySignal));
                } finally {
                    signalQueueLock.unlock();
                }
            }
        }

        private SignalMeasurementArrayQueue<Double> getDoubleMeasurementArrayQueue(
                final List<StatusSignalMeasurement<Double>> measurements
        ) {
            if (signalQueueLock.tryLock()) {
                final int signalCount = measurements.size();
                final Double[] values = new Double[signalCount];
                final double[] timestamps = new double[signalCount];

                try {
                    int i = 0;
                    for (final StatusSignalMeasurement<Double> measurement : measurements) {
                        values[i] = measurement.value;
                        timestamps[i] = measurement.bestTimestamp.getTime();
                    }
                } finally {
                    signalQueueLock.unlock();
                }

                return new SignalMeasurementArrayQueue<>(values, timestamps);
            } else {
                return new SignalMeasurementArrayQueue<>(new Double[0], new double[0]);
            }
        }

        public StatusCode[] getBadStatusCodeQueue() {
            return statusCodeBadQueue.toArray(StatusCode[]::new);
        }

        public SignalMeasurementArrayQueue<Double> getDrivePositionSignalQueue() {
            return getDoubleMeasurementArrayQueue(drivePositionSignalQueue);
        }

        public SignalMeasurementArrayQueue<Double> getDriveVelocitySignalQueue() {
            return getDoubleMeasurementArrayQueue(driveVelocitySignalQueue);
        }

        public SignalMeasurementArrayQueue<Double> getTurnPositionSignalQueue() {
            return getDoubleMeasurementArrayQueue(turnPositionSignalQueue);
        }

        public SignalMeasurementArrayQueue<Double> getTurnVelocitySignalQueue() {
            return getDoubleMeasurementArrayQueue(turnVelocitySignalQueue);
        }
    }

    public SwerveModuleIOFalconSim(
            final TalonFX driveMotor,
            final TalonFX turnMotor,
            final CANcoder turnEncoder,
            final InvertedValue driveInvertedValue,
            final InvertedValue turnInvertedValue,
            final double magnetOffset
    ) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.driveInvertedValue = driveInvertedValue;
        this.driveSim = new CTREPhoenix6TalonFXSim(
                driveMotor,
                Constants.Modules.DRIVER_GEAR_RATIO,
                new DCMotorSim(
                        SimUtils.getFalcon500FOC(1),
                        Constants.Modules.DRIVER_GEAR_RATIO,
                        Constants.Modules.DRIVE_WHEEL_MOMENT_OF_INERTIA
                )
        );

        this.turnEncoder = turnEncoder;
        this.magnetOffset = magnetOffset;
        this.turnInvertedValue = turnInvertedValue;
        this.turnSim = new CTREPhoenix6TalonFXSim(
                turnMotor,
                Constants.Modules.TURNER_GEAR_RATIO,
                new DCMotorSim(
                        SimUtils.getFalcon500FOC(1),
                        Constants.Modules.TURNER_GEAR_RATIO,
                        Constants.Modules.TURN_WHEEL_MOMENT_OF_INERTIA
                )
        );
        this.turnSim.attachFeedbackSensor(new SimPhoenix6CANCoder(turnEncoder));

        this.velocityVoltage = new VelocityVoltage(0);
        this.positionVoltage = new PositionVoltage(0);

        this.deltaTime = new DeltaTime();
    }

    @Override
    public void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        // TODO: I think we need to look at VoltageConfigs and/or CurrentLimitConfigs for limiting the
        //  current we can apply in sim, this is cause we use VelocityVoltage in sim instead of VelocityTorqueCurrentFOC
        //  which means that TorqueCurrent.PeakForwardTorqueCurrent and related won't affect it
        driveTalonFXConfiguration.Slot0 = new Slot0Configs(0, 0, 0, 0.913);
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveTalonFXConfiguration.MotorOutput.Inverted = driveInvertedValue;
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        turnTalonFXConfiguration.Slot0 = new Slot0Configs(35, 0, 0, 0);
//        turnTalonFXConfiguration.Voltage.PeakForwardVoltage = 6;
//        turnTalonFXConfiguration.Voltage.PeakReverseVoltage = -6;
        turnTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = Constants.Modules.TURNER_GEAR_RATIO;
        turnTalonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnTalonFXConfiguration.MotorOutput.Inverted = turnInvertedValue;
        turnMotor.getConfigurator().apply(turnTalonFXConfiguration);

        // TODO: this fix for CANCoder initialization in sim doesn't seem to work all the time...investigate!
//      SimUtils.initializeCTRECANCoderSim(turnEncoder);
        SimUtils.setCTRETalonFXSimStateMotorInverted(driveMotor, driveInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(turnMotor, turnInvertedValue);
    }

    @Override
    public void periodic() {
        final double dtSeconds = deltaTime.get();
        driveSim.update(dtSeconds);
        turnSim.update(dtSeconds);
    }

    @Override
    @SuppressWarnings("DuplicatedCode")
    public void updateInputs(final SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = getDriveVelocity();
        inputs.driveTorqueCurrentAmps = driveMotor.getTorqueCurrent().refresh().getValue();
        inputs.driveStatorCurrentAmps = driveMotor.getStatorCurrent().refresh().getValue();
        inputs.driveTempCelsius = driveMotor.getDeviceTemp().refresh().getValue();

        inputs.turnAbsolutePositionRots = getAngle().getRotations();
        inputs.turnVelocityRotsPerSec = turnEncoder.getVelocity().refresh().getValue();
        inputs.turnTorqueCurrentAmps = turnMotor.getTorqueCurrent().refresh().getValue();
        inputs.turnStatorCurrentAmps = turnMotor.getStatorCurrent().refresh().getValue();
        inputs.turnTempCelsius = turnMotor.getDeviceTemp().refresh().getValue();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
                Phoenix6Utils.latencyCompensateIfSignalIsGood(
                        turnEncoder.getAbsolutePosition(), turnEncoder.getVelocity()
                )
        );
    }

    public double getDrivePosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(driveMotor.getPosition(), driveMotor.getVelocity());
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().refresh().getValue();
    }

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        driveMotor.setControl(velocityVoltage.withVelocity(desiredDriverVelocity));
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        if (Constants.CTRE.DISABLE_NEUTRAL_MODE_IN_SIM) {
            // just ignore setNeutralMode call if NeutralMode setting in Simulation is turned off
            return;
        }

        final StatusCode refreshCode = driveMotor.getConfigurator().refresh(turnTalonFXConfiguration);
        if (!refreshCode.isOK()) {
            // warn if the refresh call failed in sim, which might happen pretty often as
            // there seems to be an issue with calling refresh while disabled in sim
            DriverStation.reportWarning(
                    String.format(
                            "Failed to set NeutralMode on TalonFX %s (%s)",
                            driveMotor.getDeviceID(),
                            driveMotor.getNetwork()
                    ), false
            );
            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;
        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
