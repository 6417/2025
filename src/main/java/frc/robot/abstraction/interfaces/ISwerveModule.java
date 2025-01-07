package frc.robot.abstraction.interfaces;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.fridowpi.module.IModule;

/**
 * ISwerveModule
 */
public interface ISwerveModule extends IModule {
	public SwerveModulePosition getOdometryPos();
}
