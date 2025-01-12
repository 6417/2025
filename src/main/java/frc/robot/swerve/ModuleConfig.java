package frc.robot.swerve;

import edu.wpi.first.math.util.Units;

class ModuleConfig {
    public final double angleGearboxRatio = 47.62;
    public final double driveGearboxRatio = 5.192;
    public final double encoderThicksToRotationFalcon = 1;
    public final double encoderVelocityToRPSFalcon =1;
    public final double encoderThicksToRotationNEO = 1;
    public final double encoderVelocityToRPSNEO =1;
    public final double maxSpeed = 6.5;
    public final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
}
