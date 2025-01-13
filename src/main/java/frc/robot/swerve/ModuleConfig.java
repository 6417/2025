package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.FridolinsMotor;

class ModuleConfig {
    public String name;  
    public double angleGearboxRatio = 47.62;
    public double driveGearboxRatio = 5.192;
    public double encoderThicksToRotationFalcon = 1;
    public double encoderVelocityToRPSFalcon =1;
    public double encoderThicksToRotationNEO = 1;
    public double encoderVelocityToRPSNEO =1;
    public double maxSpeed = 6.5;
    public double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
    public Translation2d moduleOffset;

    FridolinsMotor angleMotor() {
        // TODO
    }

    FridolinsMotor driveMotor() {
        // TODO
    }
}
