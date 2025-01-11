package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.sensors.AnalogEncoder;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;

class SwerveModule {
    private String moduleName;
    private FridolinsMotor driveMotor;
    private FridolinsMotor angleMotor;
    private AnalogEncoder absoluteEncoder;

    private ModuleConfig config;

    private double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

    private double maxSpeed = 6.5;

    private double lastAngle;

    public SwerveModule(){

    }

    public void stopMotors(){
        driveMotor.setVoltage(.0);
        angleMotor.setVoltage(.0);
    }

    public void setIdleMode(IdleMode mode) {
        driveMotor.setIdleMode(mode);
    }
    
    public void resetToAbsolute() {
        double position = absoluteEncoder.getAbsolutePosition() * config.angleGearboxRatio * config.encoderThicksToRotationNEO;
        angleMotor.setEncoderPosition(position);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        //desiredState = CTREModuleState.optimize(desiredState, getState().angle); // minimize the change in
                                                                                 // heading/easiest way
        
        double desiredVelocity = (desiredState.speedMetersPerSecond / wheelCircumference) * config.driveGearboxRatio * config.encoderVelocityToRPSFalcon;
        driveMotor.setVelocity(desiredVelocity); // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
        
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getRotations(); // https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
        
        angleMotor.setPosition(angle);

        lastAngle = angle;

    }

    public void setDesiredStateWithPercentOutput(SwerveModuleState desiredState) {

        //desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        //minimize the change in heading/easiest way

        double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
        driveMotor.set(percentOutput);
        
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getRadians(); // https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
        
        angleMotor.setPosition(angle);

        lastAngle = angle;

    }

    public double getVelocityMPS() {
        return getVelocityRPS() * wheelCircumference;
    }

    public double getVelocityRPS() {
        return driveMotor.getEncoderVelocity() / config.encoderVelocityToRPSFalcon; 
    }

    public String getName() {
        return moduleName;
    }

    public Rotation2d getEncoderRotation() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition());
    }

    public SwerveModulePosition getPosition() {
        double position = (driveMotor.getEncoderTicks() / config.encoderThicksToRotationFalcon / config.driveGearboxRatio) * wheelCircumference;
        Rotation2d angle = Rotation2d.fromRotations(angleMotor.getEncoderTicks() / config.encoderThicksToRotationNEO / config.angleGearboxRatio);
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        double velocity = (driveMotor.getEncoderVelocity() / config.encoderVelocityToRPSFalcon / config.driveGearboxRatio) * wheelCircumference;
        Rotation2d angle = Rotation2d.fromRotations(angleMotor.getEncoderTicks() / config.encoderThicksToRotationNEO / config.angleGearboxRatio);
        return new SwerveModuleState(velocity, angle);
    }
    
}
