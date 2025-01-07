package frc.robot.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Collection;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.fridowpi.module.IModule;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.fridowpi.utils.Vector2;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BSwerveModule;

public class SwerveModule extends BSwerveModule {
    // Set variables (TODO: Intergrate into RobotData)
    public static class Config implements Cloneable {
        public int absoluteEncoderChannel;
        public double absoluteEncoderZeroPosition;
        public Supplier<FridolinsMotor> driveMotorInitializer;
        public Supplier<FridolinsMotor> rotationMotorInitializer;
        public PidValues drivePID;
        public FeedForwardValues driveFeedForward;
        public double driveAccelerationLimitRotationsPerS2;
        public PidValues rotationPID;
        public double rotationMotorTicksPerRotation;
        public double driveMotorTicksPerRotation;
        public double wheelCircumference; // in meter
        public Translation2d mountingPoint; // in meter
        public double maxVelocity; // in drive motor encoder velocity units
        public FridoFeedBackDevice driveEncoderType;
        public FridoFeedBackDevice rotationEncoderType;
        public Optional<Boolean> driveSensorInverted = Optional.empty();
        public boolean driveMotorInverted;
        public IdleMode driveIdleMode;
        public LimitSwitchPolarity limitSwitchPolarity;
        public IdleMode rotationIdleMode;

        @Override
        public Config clone() {
            try {
                return (Config) super.clone();
            } catch (CloneNotSupportedException e) {
                Config copy = new Config();
                copy.absoluteEncoderChannel = absoluteEncoderChannel;
                copy.absoluteEncoderZeroPosition = absoluteEncoderZeroPosition;
                copy.driveMotorInitializer = driveMotorInitializer;
                copy.rotationMotorInitializer = rotationMotorInitializer;
                copy.drivePID = drivePID.clone();
                copy.driveFeedForward = driveFeedForward.clone();
                copy.driveAccelerationLimitRotationsPerS2 = driveAccelerationLimitRotationsPerS2;
                copy.rotationPID = rotationPID.clone();
                copy.rotationMotorTicksPerRotation = rotationMotorTicksPerRotation;
                copy.driveMotorTicksPerRotation = driveMotorTicksPerRotation;
                copy.wheelCircumference = wheelCircumference;
                copy.mountingPoint = new Translation2d(mountingPoint.getX(), mountingPoint.getY());
                copy.driveMotorInverted = driveMotorInverted;
                copy.driveIdleMode = driveIdleMode;
                copy.rotationIdleMode = rotationIdleMode;
                return copy;
            }
        }
    }

    private class Motors {
        public FridolinsMotor drive;
        public FridolinsMotor rotation;
        public AnalogEncoder absoluteEncoder;

        public Motors(Config config) {
            drive = config.driveMotorInitializer.get();
            drive.configEncoder(config.driveEncoderType, (int) config.driveMotorTicksPerRotation);
            config.driveSensorInverted.ifPresent(drive::setEncoderDirection);
            drive.setInverted(config.driveMotorInverted);
            drive.setPID(config.drivePID, config.driveFeedForward);

            rotation = config.rotationMotorInitializer.get();
            rotation.configEncoder(config.rotationEncoderType, (int) config.rotationMotorTicksPerRotation);
            rotation.setIdleMode(IdleMode.kCoast);
            rotation.setPID(config.rotationPID);

            absoluteEncoder = new AnalogEncoder(config.absoluteEncoderChannel);
            absoluteEncoder.setPositionOffset(config.absoluteEncoderZeroPosition);
            zeroRelativeEncoder();
        }

        public void zeroRelativeEncoder() {
            var pos = absoluteEncoder.get();
            var newPos = pos * config.rotationMotorTicksPerRotation;
            rotation.setEncoderPosition(newPos);
        }
    }

    private Motors motors;
    public Config config;
    private SwerveModuleState desiredState = new SwerveModuleState();
    public boolean currentRotationInverted = false;

    public SwerveModule(Config config) {
        this.config = config;
        motors = new Motors(this.config);
    }

    public Vector2 getModuleRotation() {
        return Vector2.fromRadians(getModuleRotationAngle());
    }

    public void zeroRelativeEncoder() {
        motors.zeroRelativeEncoder();
    }

    public double getModuleRotationAngle() {
        return Vector2
                .fromRadians(((motors.rotation.getEncoderTicks() / config.rotationMotorTicksPerRotation) * Math.PI * 2)
                        % (Math.PI * 2))
                .getAngleAsRadians();
    }

    public double getRawModuleRotationAngle() {
        return (motors.rotation.getEncoderTicks() / config.rotationMotorTicksPerRotation) * Math.PI * 2;
    }

    public Vector2 getTargetVector() {
        return Vector2.fromRadians(desiredState.angle.getRadians());
    }

    private double angleToRotationMotorEncoderTicks(double angle) {
        double angleDelta = Math.acos(getModuleRotation().dot(Vector2.fromRadians(angle)));
        if (currentRotationInverted)
            angleDelta = Math.PI * 2 + angleDelta;
        double steeringDirection = Math.signum(getModuleRotation().cross(Vector2.fromRadians(angle)));
        return motors.rotation.getEncoderTicks()
                + steeringDirection * (angleDelta / (Math.PI * 2)) * config.rotationMotorTicksPerRotation;
    }

    private double velocity2driveMotorEncoderVelocityUnits(double speed) {
        return speed
                / config.wheelCircumference
                / Constants.SwerveDrive.Swerve2024.gearRatio;
    }

    @SuppressWarnings("unused")
    private double driveMotorEncoderVelocityToPercent(double encoderSpeed) {
        return encoderSpeed / velocity2driveMotorEncoderVelocityUnits(config.maxVelocity);
    }

    public void setDesiredState(SwerveModuleState state) {
        var dst = Vector2.fromRadians(state.angle.getRadians());
        var src = getModuleRotation();
        if (src.dot(dst) < 0) {
            state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(180));
            state.speedMetersPerSecond *= -1;
        }
        this.desiredState = state;
    }

    double encoderVel = 0.0;

    @Override
    public void driveForward(double speedFactor) {
        double vel = speedFactor * desiredState.speedMetersPerSecond;
        encoderVel = velocity2driveMotorEncoderVelocityUnits(vel);
        motors.rotation.setPosition(angleToRotationMotorEncoderTicks(desiredState.angle.getRadians()));
        ((FridoFalcon500v6) motors.drive).asTalonFX().setControl(new VelocityVoltage(encoderVel));
    }

    public void setDriveMotorSpeed(double velocity) {
        motors.drive.setVelocity(velocity);
    }

    @Override
    public void rotate(double speed) {
        motors.rotation.set(speed);
    }

    public void setDesiredRotationMotorTicks(double position) {
        motors.rotation.setPosition(position);
    }

    public double getRotationEncoderTicks() {
        return motors.rotation.getEncoderTicks();
    }

    public double getAbsoluteEncoderTicks() {
        return motors.absoluteEncoder.get();
    }

    public void stopAllMotors() {
        motors.drive.set(0.0);
        motors.rotation.set(0.0);
    }

    public void setCurrentRotationToEncoderHome() {
        motors.rotation.setEncoderPosition(0);
        motors.absoluteEncoder.setPositionOffset(motors.absoluteEncoder.getAbsolutePosition());
    }

    public void invertRotationDirection() {
        currentRotationInverted = !currentRotationInverted;
    }

    // Getters and setters
    public SwerveModuleState getDesiredModuleState() {
        return desiredState;
    }

    IdleMode mode = IdleMode.kCoast;
    double absPos = 0;

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.addDoubleProperty("Error   : ", () -> motors.drive.getEncoderVelocity() - encoderVel, null);
        builder.addDoubleProperty("Desired speed", () -> desiredState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Desired angle", desiredState.angle::getDegrees, null);
        builder.addDoubleProperty("Current speed", this::getWheelSpeed, null);
        builder.addDoubleProperty("Current angel", () -> getModuleRotationAngle() * 180 / Math.PI, null);
        builder.addDoubleProperty("Rotation Encoder Ticks", motors.rotation::getEncoderTicks, null);
        builder.addDoubleProperty("Encoder Ticks", motors.drive::getEncoderTicks, null);
        builder.addDoubleProperty("Absolute Encoder", motors.absoluteEncoder::get, null);
        builder.addDoubleProperty("Absolute Pos", motors.absoluteEncoder::getAbsolutePosition, null);
        builder.addDoubleProperty("Velocity Error",
                () -> Math.abs(getWheelSpeed()) - Math.abs(desiredState.speedMetersPerSecond * 0.075), null);
        builder.addDoubleProperty("Set abs enc", () -> absPos, val -> {
            absPos = val;
            motors.absoluteEncoder.reset();
        });
        // motors.absoluteEncoder.setPositionOffset(val);});
        builder.addBooleanProperty("Zero Module", () -> false, _ignore -> zeroRelativeEncoder());

        builder.addDoubleProperty("Target", motors.rotation::getPidTarget, null);
        builder.addBooleanProperty("Coast", () -> mode == IdleMode.kCoast,
                val -> mode = val ? IdleMode.kCoast : IdleMode.kBrake);

        builder.addDoubleProperty("WheelSpeeds", this::getWheelSpeed, null);
        builder.addDoubleProperty("moduleposonfield", () -> getOdometryPos().distanceMeters, null);
        builder.addDoubleProperty("rotangle", () -> getOdometryPos().angle.getDegrees(), null);
    }

    @Override
    public void zeroAbsoluteEncoder() {
        motors.absoluteEncoder.reset();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        /* This function shell not be used at all */
        motors.drive.setIdleMode(mode);
        motors.rotation.setIdleMode(mode);
        this.mode = mode;
    }

    @Override
    public SwerveModulePosition getOdometryPos() {
        return new SwerveModulePosition(
                motors.drive.getEncoderTicks() *
                        Constants.SwerveDrive.Swerve2024.gearRatio *
                        config.wheelCircumference,
                new Rotation2d(Radians.of(getModuleRotationAngle())));
    }

    @Override
    public Collection<IModule> getAllSubModules() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAllSubModules'");
    }

    @Override
    public Collection<IModule> getSubModules() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSubModules'");
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public boolean isInitialized() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isInitialized'");
    }

    @Override
    public double getWheelSpeed() {
        return motors.drive.getEncoderVelocity() // Rotations per second
                * Constants.SwerveDrive.Swerve2024.gearRatio // Wheel rotations per second
                * config.wheelCircumference; // Meters per second
    }

    @Override
    public FridolinsMotor getDriveMotor() {
        return motors.drive;
    }

    @Override
    public FridolinsMotor getRotationMotor() {
        return motors.rotation;
    }

    @Override
    public void registerSubmodule(IModule... subModule) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'registerSubmodule'");
    }
}
