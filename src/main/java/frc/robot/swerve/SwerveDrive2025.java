// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridolinsMotor;
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

  public AHRS mGyro = new AHRS(SPI.Port.kMXP);

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

  public static SwerveDrive2025 getInstance() {
    if (mInstance == null) {
      mInstance = new SwerveDrive2025();
    }
    return mInstance;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public synchronized void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    /* Automatically correcting the heading based on pid */

    // if robot is field centric, construct ChassisSpeeds from field relative speeds
    // if not, construct ChassisSpeeds from robot relative speeds
    ChassisSpeeds velocity = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,new Rotation2d())
            //getDriverCentricRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    desiredChassisSpeeds = ChassisSpeeds.discretize(velocity, 0.02);

    states = DriveConstants.kinematics.toSwerveModuleStates(desiredChassisSpeeds);

    if (isLocked) {
      states = new SwerveModuleState[] {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);

    /*
     * Sets open loop states
     */
    for (int i = 0; i < 4; i++) {
      mSwerveModules[i].setDesiredState(states[i], true);
    }

    // logging may be?;
  }

  /*public void resetOdometry(Pose2d pose) {
    // mGyro.reset();
    // mGyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro ? -1 :
    // 1).getDegrees());
    mOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    // mOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry(Rotation2d angle) {
    Pose2d pose = new Pose2d(getPoseMeters().getTranslation(), angle);
    mGyro.reset();
    mGyro.setYaw(angle.getDegrees());
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }*/

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(mGyro.getYaw());
  }

  public void zeroHeading() {
    mGyro.reset();
    //setYaw(0)
  }

  public void stop() {
    for (SwerveModuleBase module : mSwerveModules) {
      module.stop();
    }
  }


  public void resetToAbsolute() {
    for (SwerveModuleBase module : mSwerveModules) {
      module.stop();
      module.resetToAbsolute();
    }
  }

  public void lockSwerve(boolean should) {
    isLocked = should;
  }

  // /*
  // * ----------- SETTERS
  // */

  public void updateOdometry() {
    mOdometry.update(
        getRotation2d(),
        getModulePositions());
  }

  public void setPose(Pose2d pose) {
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }

  /*
   * Will be used in Auto by PPSwerveControllerCommand
   */

  public synchronized void setClosedLoopStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    if (isLocked) {
      states = new SwerveModuleState[] {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }
    mSwerveModules[0].setDesiredState(desiredStates[0], false);
    mSwerveModules[1].setDesiredState(desiredStates[1], false);
    mSwerveModules[2].setDesiredState(desiredStates[2], false);
    mSwerveModules[3].setDesiredState(desiredStates[3], false);

  }

  /*
   * @param speeds The desired ChassisSpeeds
   * 
   * Outputs commands to the robot's drive motors given robot-relative
   * ChassisSpeeds
   * 
   * Namely, driveRobotRelative or drive
   * 
   */
  public void setClosedLoopStates(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
    setClosedLoopStates(desiredStates);
  }

  // /*
  // * ----------- GETTERS
  // */

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle(), 360.0))
        .times(DriveConstants.invertGyro ? -1 : 1);
  }

  // Returns gyro angle relative to alliance station
  public Rotation2d getDriverCentricRotation2d() {
    return DriverStation.getAlliance().get() == Alliance.Red
        ? Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle(), 360.0))
            .times(DriveConstants.invertGyro ? -1 : 1)
        : Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle() + 180, 360.0))
            .times(DriveConstants.invertGyro ? -1 : 1);
  }

  public Pose2d getPoseMeters() {
    return mOdometry.getPoseMeters();
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        mSwerveModules[0].getPosition(),
        mSwerveModules[1].getPosition(),
        mSwerveModules[2].getPosition(),
        mSwerveModules[3].getPosition(),
    };
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModuleBase mod : mSwerveModules) {
      states[mod.getModuleNumber()] = mod.getState();
      mod.getDriveMotor();
    }
    return states;
  }

  /*
   * @return the desired wheel speeds in meters per second.
   *
   * Namely, getRobotRelativeSpeeds or getCurrentSpeeds
   */
  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kinematics.toChassisSpeeds(mSwerveModules[0].getState(), mSwerveModules[1].getState(),
        mSwerveModules[2].getState(), mSwerveModules[3].getState());
  }

  public ArrayList<FridolinsMotor> getDriveMotors() {
    ArrayList<FridolinsMotor> motors = new ArrayList<FridolinsMotor>();
    for (SwerveModuleBase module : mSwerveModules) {
      motors.add(module.getDriveMotor());
    }
    return motors;
  }

}
