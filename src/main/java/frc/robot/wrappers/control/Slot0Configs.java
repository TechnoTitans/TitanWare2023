package frc.robot.wrappers.control;

public class Slot0Configs extends com.ctre.phoenix6.configs.Slot0Configs {
    /**
     * @see com.ctre.phoenix6.configs.Slot0Configs
     * @param kP see {@link com.ctre.phoenix6.configs.Slot0Configs#kP}
     * @param kD see {@link com.ctre.phoenix6.configs.Slot0Configs#kD}
     * @param kS see {@link com.ctre.phoenix6.configs.Slot0Configs#kS}
     * @param kV see {@link com.ctre.phoenix6.configs.Slot0Configs#kV}
     */
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
