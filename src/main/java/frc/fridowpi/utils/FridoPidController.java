
package frc.fridowpi.utils;

import edu.wpi.first.math.controller.PIDController;

/**
 * FridoPidController
 */
public class FridoPidController extends PIDController {
	private double positionTolerance = 0.01;
	private double velocityTolerance = 0.01;

	public FridoPidController(double kP, double kI, double kD) {
		super(kP, kI, kD);
		setTolerance(positionTolerance, velocityTolerance);
	}

	public FridoPidController(double kP, double kI, double kD, double iZone) {
		super(kP, kI, kD);
		setIZone(iZone);
		setTolerance(positionTolerance, velocityTolerance);
	}

	// Getters and setters //
	public double getPositionTolerance() {
		return positionTolerance;
	}

	public void setPositionTolerance(double positionTolerance) {
		this.positionTolerance = positionTolerance;
		super.setTolerance(positionTolerance, velocityTolerance);
	}

	public double getVelocityTolerance() {
		return velocityTolerance;
	}

	public void setVelocityTolerance(double velocityTolerance) {
		this.velocityTolerance = velocityTolerance;
		super.setTolerance(positionTolerance, velocityTolerance);
	}
}
