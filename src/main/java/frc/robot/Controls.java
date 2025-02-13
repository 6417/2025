package frc.robot;

import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralDispenser;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CoralAlgaeOutCommandGroup;
import frc.robot.commands.CoralHeightPitchCommandGroup;
import frc.robot.swerve.FridoPathplanner;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.AlgaeInCommandGroup;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls implements Sendable {

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

    public enum Climberstate {
        kForward,
        kSteady,
        kBack;
    }

    public enum ControlMode {
        CONVENTIONAL,
        SEPARATE_ACCELERATION;
    }

    public enum HubturmState {
        LONE,
        LTWO,
        LTHREE,
        LFOUR,
        STATION,
        ALGAE1,
        ALGAE2;
    }

    public enum GamePieceState {
        CORAL,
        ALGUE;
    }

    public enum IntakeState {
        INTAKE,
        OUTTAKE;
    }

    private int liftingTowerState(HubturmState state) {
        switch (state) {
            case LONE:
                return CoralDispenser.l1State;
            case LTWO:
                return CoralDispenser.l2State;
            case LTHREE:
                return CoralDispenser.l3State;
            case LFOUR:
                return CoralDispenser.l4State;
            case STATION:
                return CoralDispenser.stationState;
            case ALGAE1:
                return CoralDispenser.algae1State;
            case ALGAE2:
                return CoralDispenser.algae2State;

            default:
                return -1;
        }
    }

    private GamePieceState activePieceState = GamePieceState.CORAL;
    private IntakeState activeIntakeState = IntakeState.OUTTAKE;

    public void setActivePieceState(GamePieceState newPieceState) {
        activePieceState = newPieceState;
    }

    public void updateStateControlls() {
        // not used
    }

    public void setActiveIntakeState(IntakeState state) {
        activeIntakeState = state;
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
            DriveSpeed.FAST, 0.9,
            DriveSpeed.SLOW, 0.3);
    private DriveSpeed activeSpeedFactor = DriveSpeed.SLOW;
    private double accelerationSensitivity = speedFactors.get(activeSpeedFactor);

    public static double deadBandDrive = 0.08;
    public static double deadBandTurn = 0.08;
    public boolean inputsSquared = false;

    public boolean slewRateLimited = true;
    public double slewRateLimit = 1.0;

    public double turnSensitivity = 0.08;

    public DriveOrientation driveOrientation = DriveOrientation.FieldOriented;
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
        rbButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagLeftToReef));
        lbButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagRightToReef));
        yButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagCenterToReef));

        burgerButtonDrive.onTrue(new InstantCommand(()-> RobotContainer.gyro.reset()));

        // climber; needs testing!
        xButtonDrive.onTrue(new ClimberCommand(Constants.ClimberSubsystem.positionFront, Climberstate.kForward));
        bButtonDrive.onTrue(new ClimberCommand(Constants.ClimberSubsystem.positionBack, Climberstate.kBack));
        aButtonDrive.onTrue(new ClimberCommand(Constants.ClimberSubsystem.positionSteady, Climberstate.kSteady));

        // liftingtower
        pov0Operator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerState(HubturmState.STATION)));
        pov2Operator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerState(HubturmState.ALGAE2)));
        pov6Operator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerState(HubturmState.ALGAE1)));
        yButtonOperator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerState(HubturmState.LONE)));
        bButtonOperator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerState(HubturmState.LTWO)));
        aButtonOperator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerState(HubturmState.LTHREE)));
        xButtonOperator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerState(HubturmState.LFOUR)));

        // coral handling
        lbButtonOperator.onTrue(new InstantCommand(() -> {
            setActivePieceState(GamePieceState.ALGUE);
        }));
        rbButtonOperator.onTrue(new InstantCommand(() -> {
            setActivePieceState(GamePieceState.CORAL);
        }));

        rtButtonDrive.onTrue(new CoralAlgaeOutCommandGroup());
        
        switch (activePieceState) {
            case CORAL:
                ltButtonDrive.onTrue(new CoralIntake());
                break;

            case ALGUE:
                ltButtonDrive.onTrue(new AlgaeInCommandGroup());
                break;
        }

        //xButtonDrive.onTrue(RobotContainer.pathplanner.getAutonomousSinglePathCommand("Path1m"));
        //bButtonDrive.onTrue(RobotContainer.pathplanner.getAutonomousSinglePathCommand("Path5m"));
        aButtonDrive.onTrue(new InstantCommand(()->RobotContainer.drive.resetModulesToAbsolute()).withTimeout(0.01));
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
