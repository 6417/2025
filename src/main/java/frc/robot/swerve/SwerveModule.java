package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.sensors.AnalogEncoder;
import frc.fridowpi.utils.Vector2;
import frc.robot.Utils;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;

public class SwerveModule implements Sendable {
    private String moduleName;
    private FridolinsMotor driveMotor;
    private FridolinsMotor angleMotor;
    private AnalogEncoder absoluteEncoder;

    private ModuleConfig config;

    private Rotation2d lastAngle;

    public SwerveModule(ModuleConfig config) {
        this.config = config;
        absoluteEncoder = config.makeAbsoluteEncoder();
        absoluteEncoder.setPositionOffset(config.absEncoderOffset);
        driveMotor = config.makeDriveMotor();
        angleMotor = config.makeAngleMotor();

        lastAngle = getAbsEncoderRotation();

        resetToAbsolute();
    }

    public void stopMotors() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public void setIdleMode(IdleMode mode) {
        driveMotor.setIdleMode(mode);
    }

    public void resetToAbsolute() {
        double position = (1.0 - absoluteEncoder.get()) * config.encoderThicksToRotationNEO * config.angleGearboxRatio;
        angleMotor.setEncoderPosition(position);
    }

    public Rotation2d angleToTargetAngle(Rotation2d angle) {
        Vector2 moduleRot = Vector2.fromRadians(getRotation().getRadians());
        Vector2 angleHeading = Vector2.fromRadians(angle.getRadians());
        double angleDelta = Math.acos(moduleRot.dot(angleHeading));

        // doesn'tt seem to be used in the old code
        // if (currentRotationInverted)
        // angleDelta = Math.PI * 2 + angleDelta;

        double steeringDirection = Math.signum(moduleRot.cross(angleHeading));
        return Rotation2d.fromRotations(getRotation().getRotations()
                + steeringDirection * (angleDelta / (Math.PI * 2)));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = CTREModuleState.optimize(desiredState, getRotation());

        double desiredVelocity = (desiredState.speedMetersPerSecond / config.wheelCircumference)
                * config.driveGearboxRatio
                * config.encoderVelocityToRPSFalcon;
        driveMotor.setVelocity(desiredVelocity); //

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (config.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle; 

        lastAngle = angle;

        double motorPos = angleMotor.getEncoderTicks() / config.angleGearboxRatio;
        double delta = Utils.wrap(lastAngle.getRotations() - motorPos);
        angleMotor.setPosition((motorPos + delta) * config.angleGearboxRatio);
    }

    public double getVelocityMPS() {
        return getVelocityRPS() * config.wheelCircumference;
    }

    public double getVelocityRPS() {
        return driveMotor.getEncoderVelocity() / config.encoderVelocityToRPSFalcon / config.driveGearboxRatio;
    }

    public String getName() {
        return moduleName;
    }

    public Rotation2d getAbsEncoderRotation() {
        return Rotation2d.fromRotations(absoluteEncoder.get());
    }

    public Rotation2d getRotation() {
        return getState().angle;
    }

    public SwerveModulePosition getPosition() {
        double position = (driveMotor.getEncoderTicks() / config.encoderThicksToRotationFalcon
                / config.driveGearboxRatio) * config.wheelCircumference;
        Rotation2d angle = Rotation2d.fromRotations(
                angleMotor.getEncoderTicks() / config.encoderThicksToRotationNEO / config.angleGearboxRatio);
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        double velocity = (driveMotor.getEncoderVelocity() / config.encoderVelocityToRPSFalcon
                / config.driveGearboxRatio) * config.wheelCircumference;
        Rotation2d angle = Rotation2d.fromRotations(
                angleMotor.getEncoderTicks() / config.encoderThicksToRotationNEO / config.angleGearboxRatio);
        return new SwerveModuleState(velocity, angle);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle motor ticks", () -> angleMotor.getEncoderTicks(), null);
        builder.addDoubleProperty("angle motor angle [deg]",
                () -> Utils.normalizeAngleRad(getRotation().getRadians()) * 180 / Math.PI, null);
        builder.addDoubleProperty("state speed [mps]", () -> getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("drive vel [rps]", () -> getVelocityRPS(), null);
        builder.addDoubleProperty("drive vel [mps]", () -> getVelocityMPS(), null);
        builder.addDoubleProperty("abs encoder raw", () -> absoluteEncoder.getRaw(), null);
        builder.addDoubleProperty("abs encoder value", () -> absoluteEncoder.get(), null);
        builder.addDoubleProperty("last angle [deg]", () -> lastAngle.getDegrees(), null);
    }
}
