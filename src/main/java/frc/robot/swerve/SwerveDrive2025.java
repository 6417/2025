// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.swerve.SwerveUtils.SwerveModuleConstants;

public class SwerveDrive2025 extends SubsystemBase {

  // Swerve numbering:
  // 0 1
  // 2 3

  private static SwerveDrive2025 mInstance;

  public SwerveModuleBase[] mSwerveModules; // collection of modules
  private SwerveModuleState[] states; // collection of modules' states
  private ChassisSpeeds desiredChassisSpeeds; // speeds relative to the robot chassis

  public SwerveDriveOdometry mOdometry;

  private boolean isLocked = false;

  public SwerveDrive2025() {
    mSwerveModules = new SwerveModuleBase[] {
      new SwerveModuleBase(0, "FL", SwerveModuleConstants.generateModuleConstants(
          Constants.SwerveModuleFrontLeft.driveMotorID, Constants.SwerveModuleFrontLeft.angleMotorID,
          Constants.SwerveModuleFrontLeft.cancoderID, Constants.SwerveModuleFrontLeft.angleOffset,
          Constants.SwerveModuleFrontLeft.modulekS, Constants.SwerveModuleFrontLeft.modulekV),
          DriveConstants.swerveConstants),

      new SwerveModuleBase(1, "FR", SwerveModuleConstants.generateModuleConstants(
          Constants.SwerveModuleFrontRight.driveMotorID, Constants.SwerveModuleFrontRight.angleMotorID,
          Constants.SwerveModuleFrontRight.cancoderID, Constants.SwerveModuleFrontRight.angleOffset,
          Constants.SwerveModuleFrontRight.modulekS, Constants.SwerveModuleFrontRight.modulekV),
          DriveConstants.swerveConstants),

      new SwerveModuleBase(2, "RL", SwerveModuleConstants.generateModuleConstants(
          Constants.SwerveModuleRearLeft.driveMotorID, Constants.SwerveModuleRearLeft.angleMotorID,
          Constants.SwerveModuleRearLeft.cancoderID, Constants.SwerveModuleRearLeft.angleOffset,
          Constants.SwerveModuleRearLeft.modulekS, Constants.SwerveModuleRearLeft.modulekV),
          DriveConstants.swerveConstants),

      new SwerveModuleBase(3, "RR", SwerveModuleConstants.generateModuleConstants(
          Constants.SwerveModuleRearRight.driveMotorID, Constants.SwerveModuleRearRight.angleMotorID,
          Constants.SwerveModuleRearRight.cancoderID, Constants.SwerveModuleRearRight.angleOffset,
          Constants.SwerveModuleRearRight.modulekS, Constants.SwerveModuleRearRight.modulekV),
          DriveConstants.swerveConstants)
  };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
