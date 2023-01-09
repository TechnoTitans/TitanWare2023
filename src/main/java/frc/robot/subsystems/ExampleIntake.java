package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.motors.TitanFX;

enum IntakeState {
    INTAKE_OUT, INTAKE_IN, INTAKE_OUT_STANDBY
}

@SuppressWarnings("unused")
public class ExampleIntake extends SubsystemBase {
    private final TitanFX intakeMotor;
    private final Solenoid intakeSolenoid;

    public ExampleIntake(TitanFX intakeMotor, Solenoid intakeSolenoid) {
        this.intakeMotor = intakeMotor;
        this.intakeSolenoid = intakeSolenoid;
    }

    public void setState(IntakeState state) {
        switch (state) {
            case INTAKE_OUT:
                intakeSolenoid.set(true);
                intakeMotor.set(1);
                break;
            case INTAKE_IN:
                intakeSolenoid.set(false);
                intakeMotor.set(0);
                break;
            case INTAKE_OUT_STANDBY:
                intakeSolenoid.set(true);
                intakeMotor.set(0);
                break;
        }
    }
}
