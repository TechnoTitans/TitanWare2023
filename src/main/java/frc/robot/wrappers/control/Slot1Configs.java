package frc.robot.wrappers.control;

public class Slot1Configs extends com.ctre.phoenix6.configs.Slot1Configs {
    /**
     * @see com.ctre.phoenix6.configs.Slot1Configs
     * @param kP see {@link com.ctre.phoenix6.configs.Slot1Configs#kP}
     * @param kD see {@link com.ctre.phoenix6.configs.Slot1Configs#kD}
     * @param kS see {@link com.ctre.phoenix6.configs.Slot1Configs#kS}
     * @param kV see {@link com.ctre.phoenix6.configs.Slot1Configs#kV}
     */
    public Slot1Configs(
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
