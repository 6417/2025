package frc.robot.abstraction.interfaces;

import frc.fridowpi.module.IModule;

/**
 * IShooter: Interface for all shooters
 */
public interface IShooter extends IModule {

	public interface IShooterConfig {
		public int asInt();
	}

	// Executes one shooting cycle, if defined
	public void shoot(IShooterConfig configuration);

	// Sets the motor speed of the shooter
	public void setShooterSpeedPercent(double speed);

	// Must be called in a .periodic() function, so that the motors are fed
	public void run();

	public void stopMotors();

	public void enable();

	public void disable();


	// GETTERS
	public double getSpeedPercent();
}
