package frc.robot;

import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.states.SuperStructureState;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls implements Sendable {

    // private ExampleSubsystem ss = new ExampleSubsystem();
    public CommandXboxController driveJoystick = new CommandXboxController(Constants.Joystick.driveJoystickId);
    public CommandXboxController operatorJoystick = new CommandXboxController(Constants.Joystick.operatorJoystickId);

    public SuperStructureState superstructureOnState;


    Trigger ltButtonOperator = operatorJoystick.leftTrigger();
    Trigger rtButtonOperator = operatorJoystick.rightTrigger();
    Trigger lbButtonOperator = operatorJoystick.leftBumper();
    Trigger rbButtonOperator = operatorJoystick.rightBumper();
    Trigger aButtonOperator = operatorJoystick.a();
    Trigger bButtonOperator = operatorJoystick.b();
    Trigger xButtonOperator = operatorJoystick.x();
    Trigger yButtonOperator = operatorJoystick.y();
    Trigger windowsButtonOperator = operatorJoystick.back();
    Trigger burgerButtonOperator = operatorJoystick.start();
    Trigger pov0Operator = operatorJoystick.povUp();
    Trigger pov2Operator = operatorJoystick.povRight();
    Trigger pov4Operator = operatorJoystick.povDown();
    Trigger pov6Operator = operatorJoystick.povLeft();

    Trigger ltButtonDrive = driveJoystick.leftTrigger();
    Trigger rtButtonDrive = driveJoystick.rightTrigger();
    Trigger lbButtonDrive = driveJoystick.leftBumper();
    Trigger rbButtonDrive = driveJoystick.rightBumper();
    Trigger aButtonDrive = driveJoystick.a();
    Trigger bButtonDrive = driveJoystick.b();
    Trigger xButtonDrive = driveJoystick.x();
    Trigger yButtonDrive = driveJoystick.y();
    Trigger windowsButtonDrive = driveJoystick.back();
    Trigger burgerButtonDrive = driveJoystick.start();

    public enum ControlMode {
        CONVENTIONAL,
        SEPARATE_ACCELERATION;
    }

    public enum HubturmState {
        LZERO,
        LONE,
        LTWO,
        LTHREE,
    }

    public enum GamePieceState {
        CORAL,
        ALGUE;
    }

    public enum IntakeState {
        INTAKE,
        OUTTAKE;
    }

    private GamePieceState activePieceState = GamePieceState.ALGUE;
    private IntakeState activeIntakeState = IntakeState.INTAKE;

    public void updateStateControlls() {
        yButtonOperator.whileTrue(new InstantCommand(() -> {
            if (activeIntakeState == IntakeState.INTAKE && activePieceState == GamePieceState.CORAL) {
                // What happens if CoralIntakestate is Active and y is Pressed
            } else if (activeIntakeState == IntakeState.INTAKE) {
                // What happens if AlgueIntakestate is Active and y is Pressed
            } else if (activePieceState == GamePieceState.CORAL) {
                // What happens if CoralOuttakestate is Active and y is Pressed
            } else {
                // What happens if AlgueOuttakestate is Active and y is Pressed
            }
        }));
    }

    public void setActiveIntakeState(IntakeState newIntakeState) {
        activeIntakeState = newIntakeState;
    }

    public void setActivePieceState(GamePieceState newPieceState) {
        activePieceState = newPieceState;
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

    public DriveSpeed getActiveSpeedFactor() {
        return activeSpeedFactor;
    }

    public double getAccelerationSensitivity() {
        return accelerationSensitivity;
    }

    public Controls() {
        rtButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagLeftToReef));
        ltButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagRightToReef));
        yButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagCenterToReef));

        // aButtonOperator.onTrue(new InstantCommand(() ->
        // setActivePieceState(GamePieceState.ALGUE)))
        // .onFalse(new InstantCommand(() ->
        // setActivePieceState(GamePieceState.CORAL)));

        var x = HubturmState.LONE;
        x.getClass();

        HubturmState y = HubturmState.LONE;


        burgerButtonOperator.onTrue(new InstantCommand(()-> RobotContainer.gyro.reset()));

        yButtonOperator.onTrue(new InstantCommand(() -> {
            //CoralDispenserSubsystem.setAngle(superstructureOnState(HubturmState.LONE).getAngle());
            // LiftingTowerSubsystem.setHeight(superstructureOnState(HubturmState.LONE).getHeight())
        }));
        bButtonOperator.onTrue(new InstantCommand(() -> {
            // CoralDispenserSubsystem.setAngle(superstructureOnState(HubturmState.LTWO).getAngle())
            // LiftingTowerSubsystem.setHeight(superstructureOnState(HubturmState.LTWO).getHeight())
        }));
        aButtonOperator.onTrue(new InstantCommand(() -> {
            // CoralDispenserSubsystem.setAngle(superstructureOnState(HubturmState.LTHREE).getAngle())
            // LiftingTowerSubsystem.setHeight(superstructureOnState(HubturmState.LTHREE).getHeight())
        }));
        xButtonOperator.onTrue(new InstantCommand(() -> {
            // CoralDispenserSubsystem.setAngle(superstructureOnState(HubturmState.LFOUR).getAngle())
            // LiftingTowerSubsystem.setHeight(superstructureOnState(HubturmState.LFOUR).getHeight())
        }));

        Shuffleboard.getTab("Drive").add("Controls", this);
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
