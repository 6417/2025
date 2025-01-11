package frc.robot.swerve.SwerveUtils;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Constants for individual swerve modules
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int encoderID;
    public final double encoderAngleOffset;
    public final double moduleTuningkS;
    public final double moduleTuningkV;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int encoderID, double encoderAngleOffset,
            double moduleTuningkS, double moduleTuningkV) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.encoderAngleOffset = encoderAngleOffset;
        this.moduleTuningkS = moduleTuningkS;
        this.moduleTuningkV = moduleTuningkV;
    }

    public static SwerveModuleConstants generateModuleConstants(int driveMotorID, int angleMotorID, int CANCoderID,
            double angleOffset, double moduleTuningkS, double moduleTuningkV) {
        return new SwerveModuleConstants(driveMotorID, angleMotorID, CANCoderID, angleOffset, moduleTuningkS,
                moduleTuningkV);
    }
}
