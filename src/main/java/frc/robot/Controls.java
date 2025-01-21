package frc.robot;

import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.robot.commands.CoralHeightPitchCommandGroup;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls {
    public CommandXboxController driveJoystick = new CommandXboxController(Constants.Joystick.driveJoystickId);
    public CommandXboxController operatorJoystick = new CommandXboxController(Constants.Joystick.driveJoystickId);
    public Trigger natrualStateButton = operatorJoystick.a();
    public Trigger stationStateButton = operatorJoystick.a();
    public Trigger l1StateButton = operatorJoystick.a();
    public Trigger l2StateButton = operatorJoystick.b();
    public Trigger l3StateButton = operatorJoystick.x();
    public Trigger l4StateButton = operatorJoystick.y();



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
    private DriveSpeed activeSpeedFactor = DriveSpeed.DEFAULT_SPEED;
    private double accelerationSensitivity = speedFactors.get(activeSpeedFactor);

    

    public double deadBandDrive = 0.08;
    public double deadBandTurn = 0.08;
    public boolean inputsSquared = false;

    public boolean slewRateLimited = true;
    public double slewRateLimit = 1.0;

    public double turnSensitivity = 0.08;

    public double maxTurnSpeed = Constants.SwerveDrive.Swerve2024.maxTurnSpeed;
    public double maxVelocity = Constants.SwerveDrive.Swerve2024.maxVelocity;

    public ControlMode controlMode = ControlMode.CONVENTIONAL;
	public DriveOrientation driveOrientation = DriveOrientation.FieldOriented;

    public Controls() {
        natrualStateButton.onTrue(new CoralHeightPitchCommandGroup(0));
        stationStateButton.onTrue(new CoralHeightPitchCommandGroup(1));
        l1StateButton.onTrue(new CoralHeightPitchCommandGroup(2));
        l2StateButton.onTrue(new CoralHeightPitchCommandGroup(3));
        l3StateButton.onTrue(new CoralHeightPitchCommandGroup(4));
        l4StateButton.onTrue(new CoralHeightPitchCommandGroup(5));
    }


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
    }
}
