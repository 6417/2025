
package frc.robot.abstraction.baseClasses;

import java.util.List;

import frc.robot.abstraction.interfaces.IClimber;
import frc.fridowpi.module.Module;
import frc.fridowpi.motors.FridoServoMotor;

/**
 * BRopeClimber
 */
public abstract class BClimber extends Module implements IClimber {

	public class ClimberData {
		public final List<Integer> motorIds;

		public ClimberData(List<Integer> motorIds) {
			this.motorIds = motorIds;
		}
	}

	abstract public FridoServoMotor getServoLeft();
	abstract public FridoServoMotor getServoRight();

	abstract public ClimberData getData();

}
