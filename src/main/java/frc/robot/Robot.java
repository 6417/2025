// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command autoCommand;
    private final RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        robotContainer = new RobotContainer();
        FollowPathCommand.warmupCommand().schedule();

        autoCommand = new WaitCommand(15);
        

        //autoCommand = robotContainer.getAutoCommand();
        robotContainer.gyro.reset();
        Shuffleboard.getTab("CommandScheduler").add(CommandScheduler.getInstance());
        Shuffleboard.getTab("Vision").add("XYZ Distance", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {

                builder.addDoubleProperty("distanceX",
                        () -> LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getX(),
                        null);
                builder.addDoubleProperty("distanceY",
                        () -> LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getY(),
                        null);
                builder.addDoubleProperty("distanceZ",
                        () -> LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getZ(),
                        null);

                builder.addDoubleProperty("RotationX (Roll)",
                        () -> LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getRotation().getX(),
                        null);
                builder.addDoubleProperty("RotationY (Pitch)",
                        () -> LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getRotation().getY(),
                        null);
                builder.addDoubleProperty("RotationZ (Yaw)",
                        () -> LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getRotation().getZ(),
                        null);
            }
        });

        robotContainer.drive.resetModulesToAbsolute();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutoCommand();

        if (autoCommand != null) {
            autoCommand.schedule();
        }


    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        robotContainer.drive.addVisionToOdometry();
    }

    @Override
    public void teleopInit() {
        // robotContainer.pathplanner.getAutoCommandGroup("Auto").cancel();
        if(autoCommand != null) autoCommand.cancel();
        robotContainer.drive.stopMotors();
        robotContainer.drive.resetModulesToAbsolute();
        robotContainer.gyro.setYaw(robotContainer.drive.getPose().getRotation().getDegrees());
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //robotContainer.drive.addVisionToOdometry();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
