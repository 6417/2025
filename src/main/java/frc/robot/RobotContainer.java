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
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    // public static final AHRS gyroNavx; 
    public static final Pigeon2 gyro;
    public static final FridoPathplanner pathplanner;
    public static final ClimberSubsystem climber;
    //public static final LEDSubsystem leds;
    private static final SendableChooser<Command> autoChooser;
    public static final CoralDispenserSubsystem coralDispenser;
    public static final LiftingTowerSubsystem liftingTower;
    
        static {
            // gyroNavx = new AHRS(Port.kMXP); /* old */
            gyro = new Pigeon2(Constants.Gyro.gyroId);
            //leds = new LEDSubsystem();
            drive = new SwerveDrive(Constants.SwerveDrive.configs);
            climber = new ClimberSubsystem();
            pathplanner = new FridoPathplanner(drive);
            liftingTower = new LiftingTowerSubsystem();
            coralDispenser = null; //new CoralDispenserSubsystem();
            controls = new Controls();

            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto", autoChooser);
            SmartDashboard.putData(liftingTower);
            SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());
    }

    public static synchronized Rotation2d getGyroRotation2d() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        double angle = Math.IEEEremainder(inverted * gyro.getYaw().getValueAsDouble(), 360);
        return Rotation2d.fromDegrees(angle);
    }
    
    public static Command getAutoCommand(){
        return autoChooser.getSelected();
    }
}
