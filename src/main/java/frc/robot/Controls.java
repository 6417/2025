package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralDispenser;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Climber.ClimberEncoderZeroGroup;
import frc.robot.commands.LiftingTower.AutoScore;
import frc.robot.commands.LiftingTower.CoralHeightPitchCommandGroup;
import frc.robot.commands.LiftingTower.ZeroLiftingTower;
import frc.robot.commands.CoralAlgae.AlgaeOuttakeCommandGroup;
import frc.robot.commands.CoralAlgae.CoralAlgaeOuttake;
import frc.robot.commands.CoralAlgae.IntakeGroup;

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
    Trigger pov0Drive = driveJoystick.povUp();

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

    public enum IntakeState {
        INTAKE,
        OUTTAKE;
    }

    private static HubturmState liftingTowerState = HubturmState.STATION; // default state

    public HubturmState getActiveLiftingTowerState() {
        return liftingTowerState;
    }

    public IntakeState activeIntakeState = IntakeState.OUTTAKE; // default state

    public static int liftingTowerStateInt(HubturmState state) {
        switch (state) {
            case LONE:
                liftingTowerState = HubturmState.LONE;
                return CoralDispenser.l1State;
            case LTWO:
                liftingTowerState = HubturmState.LTWO;
                return CoralDispenser.l2State;
            case LTHREE:
                liftingTowerState = HubturmState.LTHREE;
                return CoralDispenser.l3State;
            case LFOUR:
                liftingTowerState = HubturmState.LFOUR;
                return CoralDispenser.l4State;
            case STATION:
                liftingTowerState = HubturmState.STATION;
                return CoralDispenser.stationState;
            case ALGAE1:
                liftingTowerState = HubturmState.ALGAE1;
                return CoralDispenser.algae1State;
            case ALGAE2:
                liftingTowerState = HubturmState.ALGAE2;
                return CoralDispenser.algae2State;

            default:
                return -1;
        }
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
            DriveSpeed.SLOW, 0.2);
    private DriveSpeed activeSpeedFactor = DriveSpeed.DEFAULT_SPEED;
    private double accelerationSensitivity = speedFactors.get(activeSpeedFactor);

    public static double deadBandDrive = 0.08;
    public static double deadBandTurn = 0.08;
    public boolean inputsSquared = false;

    public boolean slewRateLimited = true;
    public double slewRateLimit = 1.0;

    public double turnSensitivity = 0.08;

    public double dispenserOffset = 0.0;

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
                Constants.OffsetsToAprilTags.offsetToAprilTagRight));
        lbButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagLeft));

        burgerButtonDrive.onTrue(new InstantCommand(() -> RobotContainer.gyro.reset()));

        /* climber: Tested on Friday! */
        xButtonDrive.onTrue(new ClimberCommand(RobotContainer.climber, Constants.ClimberSubsystem.positionFront,
                Climberstate.kForward));
        bButtonDrive.onTrue(new ClimberCommand(RobotContainer.climber, Constants.ClimberSubsystem.positionBack,
                Climberstate.kBack));
        aButtonDrive.onTrue(new ClimberCommand(RobotContainer.climber, Constants.ClimberSubsystem.positionSteady,
                Climberstate.kSteady));
        yButtonDrive.onTrue(new ClimberEncoderZeroGroup());
      
        ltButtonOperator.onTrue(new ZeroLiftingTower(RobotContainer.liftingTower));

        // liftingtower
        pov6Operator.onTrue(new CoralHeightPitchCommandGroup(liftingTowerStateInt(HubturmState.STATION)));
        pov2Operator.toggleOnTrue(new IntakeGroup());
        pov4Operator.onTrue(new CoralHeightPitchCommandGroup(CoralDispenser.algae1State)).or(pov4Operator.onFalse(new CoralAlgaeOuttake(RobotContainer.coralDispenser).withTimeout(0.3)));
        pov0Operator.onTrue(new CoralHeightPitchCommandGroup(CoralDispenser.algae2State)).or(pov0Operator.onFalse(new CoralAlgaeOuttake(RobotContainer.coralDispenser).withTimeout(0.3)));

 //       pov4Operator.onTrue(Commands.startEnd(() -> new CoralHeightPitchCommandGroup(CoralDispenser.algae1State), () -> new CoralAlgaeOuttake(RobotContainer.coralDispenser).withTimeout(0.3), null))

        yButtonOperator.onTrue(new AutoScore(liftingTowerStateInt(HubturmState.LONE)));
        bButtonOperator.onTrue(new AutoScore(liftingTowerStateInt(HubturmState.LTWO)));
        aButtonOperator.onTrue(new AutoScore(liftingTowerStateInt(HubturmState.LTHREE)));
        xButtonOperator.onTrue(new AutoScore(liftingTowerStateInt(HubturmState.LFOUR)));

        // lbButtonOperator.whileTrue(new
        // TowerManualControl(RobotContainer.liftingTower));

        burgerButtonOperator.toggleOnTrue(new StopAllMotors());

        rtButtonDrive.whileTrue(Commands.startEnd(
                () -> {
                    activeSpeedFactor = DriveSpeed.SLOW;
                },
                () -> {
                    activeSpeedFactor = DriveSpeed.DEFAULT_SPEED;
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

        builder.addDoubleProperty("CroralDispenserPitchOffset", ()-> dispenserOffset, (newDispOffset)-> dispenserOffset = newDispOffset);

        builder.addBooleanProperty("SlewRateLimiter", () -> slewRateLimited,
                val -> slewRateLimited = val);
        builder.addDoubleProperty("SlewRate Limit", () -> slewRateLimit, null);
        builder.addBooleanProperty("SquareInputs", () -> inputsSquared, val -> inputsSquared = val);
    }

}
