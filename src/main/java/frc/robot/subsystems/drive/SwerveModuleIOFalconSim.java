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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimPhoenix6CANCoder;
import frc.robot.utils.sim.motors.CTREPhoenix6TalonFXSim;
import frc.robot.wrappers.control.Slot0Configs;

import java.util.ArrayList;
import java.util.List;
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

    // Cached StatusSignals
    private final StatusSignal<Double> _drivePosition;
    private final StatusSignal<Double> _driveVelocity;
    private final StatusSignal<Double> _driveTorqueCurrent;
    private final StatusSignal<Double> _driveStatorCurrent;
    private final StatusSignal<Double> _driveDeviceTemp;
    private final StatusSignal<Double> _turnPosition;
    private final StatusSignal<Double> _turnVelocity;
    private final StatusSignal<Double> _turnTorqueCurrent;
    private final StatusSignal<Double> _turnStatorCurrent;
    private final StatusSignal<Double> _turnDeviceTemp;

    @SuppressWarnings("unused")
    private static class StatusSignalQueue extends Thread {
        private static final int DEFAULT_CAPACITY = 8;
        private record StatusSignalMeasurement<T>(
                T value,
                StatusCode statusCode,
                Timestamp bestTimestamp,
                AllTimestamps allTimestamps
        ) {
            public StatusSignalMeasurement(final StatusSignal<T> signal) {
                this(signal.getValue(), signal.getError(), signal.getTimestamp(), signal.getAllTimestamps());
            }

            public static StatusSignalMeasurement<Double> latencyCompensatedMeasurement(
                    final StatusSignal<Double> signal,
                    final StatusSignal<Double> deltaSignal
            ) {
                return new StatusSignalMeasurement<>(
                        BaseStatusSignal.getLatencyCompensatedValue(signal, deltaSignal),
                        signal.getError(),
                        signal.getTimestamp(),
                        signal.getAllTimestamps()
                );
            }
        }

        public record SignalMeasurementArrayQueue<T>(T[] values, double[] timestamps) {}

        private final ReentrantLock signalQueueLock;
        private final List<StatusSignal<?>> statusSignals;
        private final StatusSignal<Double> drivePosition;
        private final StatusSignal<Double> driveVelocity;
        private final StatusSignal<Double> turnPosition;
        private final StatusSignal<Double> turnVelocity;

        private final List<StatusCode> statusCodeBadQueue;
        private final List<StatusSignalMeasurement<Double>> drivePositionSignalQueue;
        private final List<StatusSignalMeasurement<Double>> driveVelocitySignalQueue;
        private final List<StatusSignalMeasurement<Double>> turnPositionSignalQueue;
        private final List<StatusSignalMeasurement<Double>> turnVelocitySignalQueue;

        public StatusSignalQueue(
                final StatusSignal<Double> drivePosition,
                final StatusSignal<Double> driveVelocity,
                final StatusSignal<Double> turnPosition,
                final StatusSignal<Double> turnVelocity
        ) {
            super();

            this.signalQueueLock = new ReentrantLock();
            this.statusSignals = List.of(drivePosition, driveVelocity, turnPosition, turnVelocity);

            this.drivePosition = drivePosition;
            this.driveVelocity = driveVelocity;
            this.turnPosition = turnPosition;
            this.turnVelocity = turnVelocity;

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
                    drivePositionSignalQueue.add(StatusSignalMeasurement.latencyCompensatedMeasurement(
                            drivePosition,
                            driveVelocity
                    ));
                    driveVelocitySignalQueue.add(new StatusSignalMeasurement<>(driveVelocity));

                    turnPositionSignalQueue.add(StatusSignalMeasurement.latencyCompensatedMeasurement(
                            turnPosition,
                            turnVelocity
                    ));
                    turnVelocitySignalQueue.add(new StatusSignalMeasurement<>(turnVelocity));
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

        this._drivePosition = driveMotor.getPosition();
        this._driveVelocity = driveMotor.getVelocity();
        this._driveTorqueCurrent = driveMotor.getTorqueCurrent();
        this._driveStatorCurrent = driveMotor.getStatorCurrent();
        this._driveDeviceTemp = driveMotor.getDeviceTemp();
        this._turnPosition = turnEncoder.getAbsolutePosition();
        this._turnVelocity = turnEncoder.getVelocity();
        this._turnTorqueCurrent = turnMotor.getTorqueCurrent();
        this._turnStatorCurrent = turnMotor.getStatorCurrent();
        this._turnDeviceTemp = turnMotor.getDeviceTemp();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        // TODO: I think we need to look at VoltageConfigs and/or CurrentLimitConfigs for limiting the
        //  current we can apply in sim, this is cause we use VelocityVoltage in sim instead of VelocityTorqueCurrentFOC
        //  which means that TorqueCurrent.PeakForwardTorqueCurrent and related won't affect it
        driveTalonFXConfiguration.Slot0 = new Slot0Configs(0, 0, 0, 0.973);
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = Constants.Modules.DRIVER_GEAR_RATIO;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveTalonFXConfiguration.MotorOutput.Inverted = driveInvertedValue;
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        turnTalonFXConfiguration.Slot0 = new Slot0Configs(40, 0, 0, 0);
        turnTalonFXConfiguration.Voltage.PeakForwardVoltage = 6;
        turnTalonFXConfiguration.Voltage.PeakReverseVoltage = -6;
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

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = getDriveVelocity();
        inputs.driveTorqueCurrentAmps = _driveTorqueCurrent.refresh().getValue();
        inputs.driveStatorCurrentAmps = _driveStatorCurrent.refresh().getValue();
        inputs.driveTempCelsius = _driveDeviceTemp.refresh().getValue();

        inputs.turnAbsolutePositionRots = getRawAngle();
        inputs.turnVelocityRotsPerSec = _turnVelocity.refresh().getValue();
        inputs.turnTorqueCurrentAmps = _turnTorqueCurrent.refresh().getValue();
        inputs.turnStatorCurrentAmps = _turnStatorCurrent.refresh().getValue();
        inputs.turnTempCelsius = _turnDeviceTemp.refresh().getValue();
    }

    /**
     * Get the measured mechanism (wheel) angle of the {@link SwerveModuleIOFalconSim}, in raw units (rotations)
     * @return the measured wheel (turner) angle, in rotations
     */
    private double getRawAngle() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_turnPosition, _turnVelocity);
    }

    public double getDrivePosition() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(_drivePosition, _driveVelocity);
    }

    public double getDriveVelocity() {
        return _driveVelocity.refresh().getValue();
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
                            driveMotor.getCANBus()
                    ), false
            );
            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;
        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
