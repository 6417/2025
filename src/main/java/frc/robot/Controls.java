package frc.robot;

import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LiftingTowerSubsystem;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls implements Sendable {

    public int tagToChase = 2;

    // private ExampleSubsystem ss = new ExampleSubsystem();
    public CommandXboxController driveJoystick = new CommandXboxController(Constants.Joystick.driveJoystickId);
    public CommandXboxController operatorJoystick = new CommandXboxController(Constants.Joystick.operatorJoystickId);

    Trigger ltButtonOperator = operatorJoystick.leftTrigger();
    Trigger rtButtonOperator = operatorJoystick.rightTrigger();
    Trigger lbButtonOperator = operatorJoystick.leftBumper();
    Trigger rbButtonOperator = operatorJoystick.rightBumper();
    Trigger aButtonOperator = operatorJoystick.a();
    Trigger bButtonOperator = operatorJoystick.b();
    Trigger xButtonOperator = operatorJoystick.x();
    Trigger yButtonOperator = operatorJoystick.y();
    
    public Controls() {        
        ltButtonOperator.whileTrue(new ChaseTagCommand(RobotContainer.drive, tagToChase, Constants.OffsetsToAprilTags.offsetToAprilTagLeftToReef));
        rtButtonOperator.whileTrue(new ChaseTagCommand(RobotContainer.drive, tagToChase, Constants.OffsetsToAprilTags.offsetToAprilTagRightToReef));
        lbButtonOperator.whileTrue(new ChaseTagCommand(RobotContainer.drive, tagToChase, Constants.OffsetsToAprilTags.offsetToAprilTagCenterToReef));

        // aButtonOperator.onTrue(new ExampleCommand(ss));
        Shuffleboard.getTab("Drive").add("Controls", this);
    }

    public enum ControlMode {
        CONVENTIONAL,
        SEPARATE_ACCELERATION;
    }

    public enum DriveSpeed {
        DEFAULT_SPEED,
        FAST,
        SLOW
    }

    public enum DriveOrientation {
        FieldOriented, Forwards, Backwards
    }

    public Map<DriveSpeed, Double> speedFactors = Map.of(
            DriveSpeed.DEFAULT_SPEED, 1.0,
            DriveSpeed.FAST, 1.0,
            DriveSpeed.SLOW, 0.3);
    private DriveSpeed activeSpeedFactor = DriveSpeed.SLOW;
    private double accelerationSensitivity = speedFactors.get(activeSpeedFactor);

    public static double deadBandDrive = 0.08;
    public static double deadBandTurn = 0.08;
    public boolean inputsSquared = false;

    public boolean slewRateLimited = true;
    public double slewRateLimit = 1.0;

    public double turnSensitivity = 0.08;

    public DriveOrientation driveOrientation = DriveOrientation.Forwards;
    public ControlMode controlMode = ControlMode.CONVENTIONAL;

    public void setActiveSpeedFactor(DriveSpeed speedFactor) {
        activeSpeedFactor = speedFactor;
        accelerationSensitivity = speedFactors.get(speedFactor);
    }

    public DriveSpeed setActiveSpeedFactor() {
        return activeSpeedFactor;
    }

    public double getAccelerationSensitivity() {
        return accelerationSensitivity;
    }

    // Shuffleboard
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");

        builder.addDoubleProperty("turnSensitivity", () -> turnSensitivity,
                val -> turnSensitivity = val);

        builder.addDoubleProperty("defaultSpeedFactor", () -> speedFactors.get(DriveSpeed.DEFAULT_SPEED),
                val -> speedFactors.put(DriveSpeed.DEFAULT_SPEED, val));
        builder.addDoubleProperty("slowSpeedFactor", () -> speedFactors.get(DriveSpeed.SLOW),
                val -> speedFactors.put(DriveSpeed.SLOW, val));
        builder.addDoubleProperty("fastSpeedFactor", () -> speedFactors.get(DriveSpeed.FAST),
                val -> speedFactors.put(DriveSpeed.FAST, val));
        builder.addDoubleProperty("Current Speed Factor", () -> accelerationSensitivity, null);

        builder.addBooleanProperty("SlewRateLimiter", () -> slewRateLimited,
                val -> slewRateLimited = val);
        builder.addDoubleProperty("SlewRate Limit", () -> slewRateLimit, null);
        builder.addBooleanProperty("SquareInputs", () -> inputsSquared, val -> inputsSquared = val);

        builder.addDoubleProperty("TagToChase", ()->tagToChase, (val) -> tagToChase = (int) val);

    }
}
