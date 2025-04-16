// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.fridowpi.utils.CSVLogger;


/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer robotContainer;
    private Command autonomousCommand;
    private final CSVLogger logger = new CSVLogger("/tmp/log.csv", 10000);
    private long time;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        
        FollowPathCommand.warmupCommand().schedule();

        robotContainer = new RobotContainer();
        time = System.currentTimeMillis();

        FollowPathCommand.warmupCommand().schedule();
        robotContainer.drive.resetModulesToAbsolute();
        robotContainer.coralDispenser.resetPitchEncoder();

        Shuffleboard.getTab("Climber").add(robotContainer.climber);
        Shuffleboard.getTab("CoralHandler").add(robotContainer.coralDispenser);
        Shuffleboard.getTab("LiftingTower").add(robotContainer.liftingTower);

        RobotContainer.leds.coralL2OuttakeLEDs();

        // Creates a SysIdRoutine
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
        LimelightHelpers.SetIMUMode(Constants.Limelight.limelightID, 0);
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        //robotContainer.drive.resetOdoemetry(new Pose2d(2,7, new Rotation2d()));
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        robotContainer.drive.addVisionToOdometry();
        LimelightHelpers.SetIMUMode(Constants.Limelight.limelightID, 2);
    }

    @Override
    public void teleopInit() {
        // robotContainer.pathplanner.getAutoCommandGroup("Auto").cancel();

        robotContainer.drive.stopMotors();
        robotContainer.drive.resetModulesToAbsolute();
        robotContainer.gyro.setYaw(robotContainer.drive.getPose().getRotation().getDegrees());
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        robotContainer.drive.addVisionToOdometry();
        
        LimelightHelpers.SetIMUMode(Constants.Limelight.limelightID, 2);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        /*
         * robotContainer.liftingTower.setHeight(5);
         * robotContainer.liftingTower.setDefaultCommand(new RunCommand(() ->
         * robotContainer.liftingTower.runAutomatic(), robotContainer.liftingTower));
         */

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
