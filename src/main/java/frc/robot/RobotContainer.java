package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.LiftingTowerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.swerve.FridoPathplanner;
import frc.robot.swerve.SwerveDrive;
import frc.robot.Controls.HubturmState;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.CoralAlgae.CoralIntake;
import frc.robot.commands.LiftingTower.AutoScore;
import frc.robot.commands.LiftingTower.CoralHeightPitchCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.Map;
import java.util.HashMap;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    // public static final AHRS gyroNavx; 
    public static final Pigeon2 gyro;
    public static final FridoPathplanner pathplanner;
    public static final ClimberSubsystem climber;
    public static final LEDSubsystem leds;
    private static final SendableChooser<Command> autoChooser;
    public static final CoralDispenserSubsystem coralDispenser;
    public static final LiftingTowerSubsystem liftingTower;

    private static final Map<String, Command> namedCommands;
    
    static {
        gyro = new Pigeon2(Constants.Gyro.gyroId);
        // gyroNavx = new AHRS(Port.kMXP); /* old */
        drive = new SwerveDrive(Constants.SwerveDrive.configs);
        leds = null;//new LEDSubsystem();
        pathplanner = new FridoPathplanner(drive);
        climber = new ClimberSubsystem();
        coralDispenser = new CoralDispenserSubsystem();
        liftingTower = new LiftingTowerSubsystem();
        controls = new Controls();

        namedCommands = new HashMap<String, Command>();

        namedCommands.put("AutoScoreL4", new AutoScore(controls.liftingTowerStateInt(HubturmState.LFOUR)));
        namedCommands.put("StationLevel", new CoralHeightPitchCommandGroup(controls.liftingTowerStateInt(HubturmState.STATION)));
        namedCommands.put("CoralInput", new CoralIntake(coralDispenser));

        namedCommands.put("RightChaseTag", new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagLeftToReef));
        namedCommands.put("LeftChaseTag", new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagRightToReef));
        namedCommands.put("MidChaseTag", new ChaseTagCommand(RobotContainer.drive,
                Constants.OffsetsToAprilTags.offsetToAprilTagCenterToReef));

        pathplanner.registerCommand(namedCommands);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);
        SmartDashboard.putData(liftingTower);
        SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());
    }

    public static Rotation2d getGyroRotation2d() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        double angle = Math.IEEEremainder(inverted * gyro.getYaw().getValueAsDouble(), 360);
        return Rotation2d.fromDegrees(angle);
    }
    
    public static Command getAutoCommand(){
        return autoChooser.getSelected();
    }
}
