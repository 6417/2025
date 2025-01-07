
package frc.robot.abstraction.baseClasses;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.abstraction.interfaces.ISwerveModule;

/**
 * BSwerveModule
 */
public abstract class BSwerveModule implements Sendable, ISwerveModule {

	abstract public void driveForward(double speed);

	abstract public void rotate(double speed);

	abstract public void stopAllMotors();

	abstract public void setCurrentRotationToEncoderHome();

	abstract public double getRotationEncoderTicks();

	abstract public void setDesiredState(SwerveModuleState state);

	abstract public void setDesiredRotationMotorTicks(double position);

	abstract public void setIdleMode(IdleMode mode);

	abstract public void zeroRelativeEncoder();

	abstract public void zeroAbsoluteEncoder();

	abstract public double getWheelSpeed();

	abstract public FridolinsMotor getDriveMotor();

	abstract public FridolinsMotor getRotationMotor();
}
