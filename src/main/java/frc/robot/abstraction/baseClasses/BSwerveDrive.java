package frc.robot.abstraction.baseClasses;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Map.Entry;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.fridowpi.utils.Algorithms;
import frc.robot.Constants;
import frc.robot.swerve.SwerveKinematics;

public abstract class BSwerveDrive extends BDrive {
	// TODO: clean up here
	protected SwerveKinematics<MountingLocations> kinematics;

	public enum MotorType {
		DRIVE, ROTATE;
	}

	@Override
	public void init() {
		super.init();
		setUpSwerveKinematics(); // Must be called before setUpPoseEstimator()
	}

	private void setUpSwerveKinematics() {
		Map<MountingLocations, Translation2d> mountingPoints = Constants.SwerveDrive.Swerve2024.swerveModuleConfigs
				.entrySet().stream().map(Algorithms.mapEntryFunction(config -> config.mountingPoint))
				.collect(Collectors.toMap(Entry::getKey, Entry::getValue));
		kinematics = new SwerveKinematics<MountingLocations>(mountingPoints);
	}

	public DriveOrientation getDriveMode() {
		return DriveOrientation.Backwards;
	}

	public void setDriveMode(DriveOrientation driveMode) {
	}

	public void drive(ChassisSpeeds requesteSpeeds) {
	}

	public void rotateAllModules(double speed) {
	}

	public void setRotationToHome(MountingLocations moduleLocation) {
	}

	public void setRotationEncoderTicks(MountingLocations mountingLocation, double ticks) {
	}

	public void forEachModule(Consumer<BSwerveModule> consumer) {
	}

	public boolean isModuleZeroed(MountingLocations mountingLocation) {
		return false;
	}

	public final boolean areAllModulesZeroed() {
		for (MountingLocations location : MountingLocations.values()) {
			if (!isModuleZeroed(location)) {
				return false;
			}
		}
		return true;
	}

	public Map<MountingLocations, Boolean> getZeroedModules() {
		Map<MountingLocations, Boolean> result = new HashMap<>();
		forEachModuleEntry(
				labeledModule -> result.put(
						labeledModule.getKey(), true));
		return result;
	}

	public void forEachModuleEntry(
			Consumer<Map.Entry<MountingLocations, BSwerveModule>> consumer) {
	}

	public void withModule(MountingLocations mountingLocation, Consumer<BSwerveModule> consumer) {
	}

	@Override
	public Optional<SwerveDriveKinematics> getSwerveKinematics() {
		return Optional.of(kinematics);
	}

	@Override
	public Optional<DifferentialDriveKinematics> getDifferentialKinematics() {
		return Optional.empty();
	}

	@Override
	public Optional<DifferentialDriveWheelSpeeds> getDifferentialWheelSpeeds() {
		return Optional.empty();
	}

	// TODO: make optional
	@Override
	public double getLeftEncoderPos() {
		throw new UnsupportedOperationException("Unimplemented method 'getLeftEncoderPos'");
	}

	@Override
	public double getRightEncoderPos() {
		throw new UnsupportedOperationException("Unimplemented method 'getRightEncoderPos'");
	}

	@Override
	public final boolean isSwerve() {
		return true;
	}

	public enum SwerveMotor {
		FRONT_LEFT_DRIVE(MountingLocations.FrontLeft, MotorType.DRIVE),
		FRONT_LEFT_ROTATE(MountingLocations.FrontLeft, MotorType.ROTATE),
		FRONT_RIGHT_DRIVE(MountingLocations.FrontRight, MotorType.DRIVE),
		FRONT_RIGHT_ROTATE(MountingLocations.FrontRight, MotorType.ROTATE),
		BACK_LEFT_DRIVE(MountingLocations.BackLeft, MotorType.DRIVE),
		BACK_LEFT_ROTATE(MountingLocations.BackLeft, MotorType.ROTATE),
		BACK_RIGHT_DRIVE(MountingLocations.BackRight, MotorType.DRIVE),
		BACK_RIGHT_ROTATE(MountingLocations.BackRight, MotorType.ROTATE),
		;

		private final MountingLocations location;
		private final MotorType motorType;

		private SwerveMotor(MountingLocations location, MotorType motorType) {
			this.location = location;
			this.motorType = motorType;
		}

		public SwerveMotor from(MountingLocations location, MotorType motorType) {
			return switch (motorType) {
				case DRIVE -> driveMotorFrom(location);
				case ROTATE -> rotateMotorFrom(location);
			};
		}

		public SwerveMotor driveMotorFrom(MountingLocations location) {
			return switch (location) {
				case FrontLeft -> FRONT_LEFT_DRIVE;
				case FrontRight -> FRONT_RIGHT_DRIVE;
				case BackLeft -> BACK_LEFT_DRIVE;
				case BackRight -> BACK_RIGHT_DRIVE;
			};
		}

		public SwerveMotor rotateMotorFrom(MountingLocations location) {
			return switch (location) {
				case FrontLeft -> FRONT_LEFT_ROTATE;
				case FrontRight -> FRONT_RIGHT_ROTATE;
				case BackLeft -> BACK_LEFT_ROTATE;
				case BackRight -> BACK_RIGHT_ROTATE;
			};
		}

		// Getters
		public MountingLocations getLocation() {
			return location;
		}

		public MotorType getMotorType() {
			return motorType;
		}
	}
}
