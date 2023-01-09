package frc.robot.wrappers.control;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;

@SuppressWarnings("unused")
public final class OI {
	public static final int XBOX_A = 1;
	public static final int XBOX_B = 2;
	public static final int XBOX_X = 3;
	public static final int XBOX_Y = 4;
	public static final int XBOX_BUMPER_RIGHT = 6;
	public static final int XBOX_BUMPER_LEFT = 5;
	public static final int XBOX_BTN_SELECT = 7;
	public static final int XBOX_BTN_START = 8;

	private static final double PERCENT_DEADBAND_THRESHOLD = 0.1;

	private final static XboxController xboxMain = new XboxController(RobotMap.MainController);
	private final static XboxController xboxCo = new XboxController(RobotMap.CoController);

	// XBOX Controls

	public static XboxController getXboxMain() {
		return xboxMain;
	}
	public static XboxController getXboxCo() {
		return xboxCo;
	}

	public static double getXboxLeftYMain() { return -xboxMain.getLeftY(); }

	public static double getXboxLeftXMain() { return -xboxMain.getLeftX(); }

	public static double getXboxRightYMain() {
		return xboxMain.getRightY();
	}

	public static double getXboxRightXMain() {
		return xboxMain.getRightX();
	}

	public static double getXboxLeftTriggerMain() {
		return xboxMain.getLeftTriggerAxis();
	}

	public static double getXboxRightTriggerMain(){return xboxMain.getRightTriggerAxis(); }

	public static double getXboxLeftYCo() { return -xboxCo.getLeftY(); }

	public static double getXboxLeftXCo() { return -xboxCo.getLeftX(); }

	public static double getXboxRightYCo() {
		return xboxCo.getRightY();
	}

	public static double getXboxRightXCo() {
		return xboxCo.getRightX();
	}

	public static double getXboxLeftTriggerCo() {
		return xboxCo.getLeftTriggerAxis();
	}

	public static double getXboxRightTriggerCo(){return xboxCo.getRightTriggerAxis(); }

	public static int getPOVCo() {
		return xboxCo.getPOV();
	}

	public static double deadband(double value, double deadband) {
		if (-deadband <= value && value <= deadband) {
			value = 0;
		} else if (value > deadband) {
			value -= deadband;
			value *= (1 + deadband);
		} else if (value < -deadband) {
			value += deadband;
			value *= (1 + deadband);
		}
		return value;
	}

}
