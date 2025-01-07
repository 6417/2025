package frc.robot.abstraction.baseClasses;

import java.util.List;

import frc.fridowpi.module.Module;
import frc.robot.abstraction.interfaces.IShooter;

/**
 * BShooter: Base class for all shooter subsystems
 */
public abstract class BShooter extends Module implements IShooter {

	@Override
	public void init() {
		super.init();
	}

	public static class ShooterData {
		public final List<Double> speeds;
		public final List<Integer> motorIds;
		public final int countsPerRevolution;

		public ShooterData(List<Integer> motorIds, List<Double> speeds, int countsPerRevolution) {
			this.motorIds = motorIds;
			this.speeds = speeds;
			this.countsPerRevolution = countsPerRevolution;
		}
	}

	abstract public ShooterData getData();
}
