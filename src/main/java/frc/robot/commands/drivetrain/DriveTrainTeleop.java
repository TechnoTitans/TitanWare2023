package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.profiler.Profiler;
import frc.robot.subsystems.TankDrive;
import frc.robot.utils.MathMethods;

import java.util.function.DoubleSupplier;

@SuppressWarnings("unused")
public class DriveTrainTeleop extends CommandBase {
	private final DoubleSupplier leftInput, rightInput, steeringInput;
	private SlewRateLimiter leftFilter, rightFilter;

	private final double throttleSensitivity = Profiler.getProfile().ThrottleSensitivity;
	private final double steeringSensitivity = Profiler.getProfile().SteeringSensitivity;

	private final TankDrive driveTrain;

	public DriveTrainTeleop(TankDrive driveTrain, DoubleSupplier leftInput, DoubleSupplier rightInput, DoubleSupplier steeringInput, boolean filterEnabled) {

		this.driveTrain = driveTrain;
		this.leftInput = leftInput;
		this.rightInput = rightInput;
		this.steeringInput = steeringInput;

		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		leftFilter = new SlewRateLimiter(1); // todo adjust sensitivity
		rightFilter = new SlewRateLimiter(1); // todo adjust sensitivity
	}

	@Override
	public void execute() {
		double steering = steeringInput.getAsDouble() * steeringSensitivity;
		if (MathMethods.withinBand(steering, -.25, .25)) {
			steering = 0;
		} else if (steering > 0) {
			steering = steering - .25;
		} else {
			steering = steering + .25;
		}
		double throttle = rightInput.getAsDouble() - leftInput.getAsDouble();
		double rpower =  throttle + steering;
		double lpower = throttle - steering;
		driveTrain.set(leftFilter.calculate(lpower*throttleSensitivity), rightFilter.calculate(rpower*throttleSensitivity));
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.driveTrain.stop();
	}


}
