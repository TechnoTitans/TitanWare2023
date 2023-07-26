package frc.robot.utils.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

import static com.revrobotics.CANSparkMax.ControlType;

public class RevUtils {
    public static boolean revErrorIsOk(final REVLibError revLibError) {
        return revLibError == REVLibError.kOk;
    }

    public static void reportIfNotOk(final CANSparkMax sparkMax, final REVLibError revLibError) {
        if (!revErrorIsOk(revLibError)) {
            DriverStation.reportError(
                    String.format(
                            "Failed on SparkMAX %d: %s",
                            sparkMax.getDeviceId(),
                            revLibError
                    ),
                    false
            );
        }
    }

    public static double getSparkMAXMotorVoltage(
            final CANSparkMax sparkMax,
            final CANSparkMax.ControlType controlType
    ) {
        final double busVoltage = sparkMax.getBusVoltage();
        final double appliedOutput = sparkMax.getAppliedOutput();
        return switch (controlType) {
            case kDutyCycle -> busVoltage * MathUtil.clamp(appliedOutput, -1, 1);
            case kVoltage -> busVoltage * MathUtil.clamp(appliedOutput / 12, -1, 1);
            default -> throw new UnsupportedOperationException("not implemented yet!");
        };
    }

    public static double convertControlTypeOutput(
            final CANSparkMax sparkMax,
            final CANSparkMax.ControlType controlType,
            final CANSparkMax.ControlType desiredControlType,
            final double originalInput
    ) {
        if (controlType == ControlType.kDutyCycle && desiredControlType == ControlType.kVoltage) {
            return MathUtil.clamp(originalInput * sparkMax.getBusVoltage(), -12, 12);
        } else if (controlType == ControlType.kVoltage && desiredControlType == ControlType.kDutyCycle) {
            return MathUtil.clamp(originalInput / sparkMax.getBusVoltage(), -1, 1);
        } else {
            throw new UnsupportedOperationException("not implemented yet!");
        }
    }
}
