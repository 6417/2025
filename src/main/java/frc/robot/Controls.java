package frc.robot;

import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.module.Module;
import frc.robot.abstraction.baseClasses.BDrive.SpeedFactor;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls extends Module {
	public static final Controls instance = new Controls();

	public static enum ControlMode {
		CONVENTIONAL,
		SEPARATE_ACCELERATION; // Nice controls: Joystick for direction only
	}

	public static Map<SpeedFactor, Double> speedFactors = Map.of(
			SpeedFactor.DEFAULT_SPEED, 1.0,
			SpeedFactor.FAST, 1.0,
			SpeedFactor.SLOW, 0.3);
	private static SpeedFactor activeSpeedFactor = SpeedFactor.DEFAULT_SPEED;
	private static double accelerationSensitivity = speedFactors.get(activeSpeedFactor);
	private static double deadBandDrive = 0.08;
	private static double deadBandTurn = 0.08;
	private static boolean inputsSquared = false;

	private static boolean slewRateLimited = true;
	private static double slewRateLimit = 1.0;

	private static double turnSensitivity = 0.08;
	private static ControlMode controlMode = ControlMode.CONVENTIONAL;

	public static void setActiveSpeedFactor(SpeedFactor speedFactor) {
		activeSpeedFactor = speedFactor;
		accelerationSensitivity = speedFactors.get(speedFactor);
	}

	// Getters and setters
	public static double getAccelerationSensitivity() {
		return accelerationSensitivity;
	}

	public static double getTurnSensitivity() {
		return turnSensitivity;
	}

	public static SpeedFactor getActiveSpeedFactor() {
		return activeSpeedFactor;
	}

	public static ControlMode getControlMode() {
		return controlMode;
	}

	public static void setControlMode(ControlMode controlMode) {
		Controls.controlMode = controlMode;
	}

	public static Map<SpeedFactor, Double> getSpeedFactors() {
		return speedFactors;
	}

	public static void setSpeedFactors(Map<SpeedFactor, Double> speedFactors) {
		Controls.speedFactors = speedFactors;
	}

	public static double getDeadBandDrive() {
		return deadBandDrive;
	}

	public static void setDeadBandDrive(double deadBandDrive) {
		Controls.deadBandDrive = deadBandDrive;
	}

	public static double getDeadBandTurn() {
		return deadBandTurn;
	}

	public static void setDeadBandTurn(double deadBandTurn) {
		Controls.deadBandTurn = deadBandTurn;
	}

	public static boolean isSlewRateLimited() {
		return slewRateLimited;
	}

	public static void setSlewRateLimited(boolean slewRateLimited) {
		Controls.slewRateLimited = slewRateLimited;
	}

	public static double getSlewRateLimit() {
		return slewRateLimit;
	}

	public static void setSlewRateLimit(double slewRateLimit) {
		Controls.slewRateLimit = slewRateLimit;
	}

	public static void setAccelerationSensitivity(double accelerationSensitivity) {
		Controls.accelerationSensitivity = accelerationSensitivity;
	}

	public static void setTurnSensitivity(double turnSensitivity) {
		Controls.turnSensitivity = turnSensitivity;
	}

	public static boolean isInputsSquared() {
		return inputsSquared;
	}

	public static void setInputsSquared(boolean inputsSquared) {
		Controls.inputsSquared = inputsSquared;
	}


	// Shuffleboard
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Motor Controller");

		builder.addDoubleProperty("turnSensitivity", () -> turnSensitivity,
				val -> turnSensitivity = val);

		builder.addDoubleProperty("defaultSpeedFactor", () -> speedFactors.get(SpeedFactor.DEFAULT_SPEED),
				val -> speedFactors.put(SpeedFactor.DEFAULT_SPEED, val));
		builder.addDoubleProperty("slowSpeedFactor", () -> speedFactors.get(SpeedFactor.SLOW),
				val -> speedFactors.put(SpeedFactor.SLOW, val));
		builder.addDoubleProperty("fastSpeedFactor", () -> speedFactors.get(SpeedFactor.FAST),
				val -> speedFactors.put(SpeedFactor.FAST, val));
		builder.addDoubleProperty("Current Speed Factor", () -> accelerationSensitivity, null);

		builder.addBooleanProperty("SlewRateLimiter", () -> Controls.isSlewRateLimited(), val -> Controls.setSlewRateLimited(val));
		builder.addDoubleProperty("SlewRate Limit", ()-> slewRateLimit, null);
		builder.addBooleanProperty("SquareInputs", () -> Controls.isInputsSquared(), val -> Controls.setInputsSquared(val));
	}
}
