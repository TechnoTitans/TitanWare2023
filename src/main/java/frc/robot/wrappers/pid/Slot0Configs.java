package frc.robot.wrappers.pid;

public class Slot0Configs extends com.ctre.phoenix6.configs.Slot0Configs {
    public Slot0Configs(
            final double kP,
            final double kD,
            final double kS,
            final double kV
    ) {
        super();

        super.kP = kP;
        super.kD = kD;
        super.kS = kS;
        super.kV = kV;
    }
}
