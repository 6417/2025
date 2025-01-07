package frc.robot.abstraction.interfaces;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.fridowpi.joystick.JoystickBindable;
import frc.fridowpi.module.IModule;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.abstraction.baseClasses.BDrive.DriveOrientation;

public interface IDrive extends IModule, Sendable, JoystickBindable {

    // Steering functions
    public void drive(double v_x, double v_y, double rot);
    public void drive(ChassisSpeeds chassisSpeeds);

    public void setVolts(double leftvolts, double rigthvolts);

    public void driveToPos(Pose2d pos);

	public DriveOrientation getOrientation();
	public void setOrientation(DriveOrientation orientation);

	// Factor that scales the driving speed (0..1)
	public void setSpeedFactor(double speedFactor);

    // Brake
	public void stopAllMotors();

    public void setIdleMode(IdleMode mode);

	// public void resetOdometry();

    // Sysid tuning (not really necessary)
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    public Command sysIdDynamic(SysIdRoutine.Direction direction);

    // Getters
	public <T> FridolinsMotor getMotor(T motor);

    // public Pose2d getPos();

    public SwerveModulePosition[] getModulePositions(); // For swerve drive: the positions of all modules

    public double getLeftEncoderPos();

    public double getRightEncoderPos();

	public void zeroAbsoluteEncoders();
	public void zeroRelativeEncoders();

	public double percent2rotationVelocity(double val);
	public double percent2rotationVelocityDouble(double val);

	public double percent2driveVelocity(double x);


    // Couldn't (yet) generalize these:
	//
	// Tankdrive only
    public Optional<DifferentialDriveKinematics> getDifferentialKinematics();
	public Optional<DifferentialDriveWheelSpeeds> getDifferentialWheelSpeeds();

	// Swerve only
    public Optional<SwerveDriveKinematics> getSwerveKinematics();
	public double getSwerveWheelSpeeds();

    // Abstraction stuff
    public boolean isSwerve();

    // Must be sendable
    public void initSendable(SendableBuilder builder);
}
