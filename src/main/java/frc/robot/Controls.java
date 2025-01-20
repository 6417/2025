package frc.robot;

import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChaseTagCommand;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls implements Sendable {
    public Joystick driveJoystick = new Joystick(Constants.Joystick.driveJoystickId);
    public Joystick operatorJoystick = new Joystick(Constants.Joystick.operatorJoystickId);
    public int tagToChase = 2;
    
    public Controls() {
        JoystickButton aButton = new JoystickButton(operatorJoystick, 1);
        JoystickButton bButton = new JoystickButton(operatorJoystick, 2);
        JoystickButton xButton = new JoystickButton(operatorJoystick, 3);
        JoystickButton yButton = new JoystickButton(operatorJoystick, 4);
        JoystickButton lbButton = new JoystickButton(operatorJoystick, 5);
        JoystickButton rbButton = new JoystickButton(operatorJoystick, 6);
        
        lbButton.whileTrue(new ChaseTagCommand(RobotContainer.drive, 2, Constants.OffsetsToAprilTags.offsetToAprilTagCenterToReef));
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
