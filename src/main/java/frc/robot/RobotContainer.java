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
import frc.robot.swerve.FridoPathplanner;
import frc.robot.swerve.SwerveDrive;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    // public static final AHRS gyroNavx; 
    public static final Pigeon2 gyro;
    public static final FridoPathplanner pathplanner;
    private static final SendableChooser<Command> autoChooser;
    pu
    
    public static final ClimberSubsystem climber = new ClimberSubsystem();
    // public static final CoralDispenserSubsystem coralDispenser = new CoralDispenserSubsystem();
    public static final LiftingTowerSubsystem liftingTower;
    
        static {
            // gyroNavx = new AHRS(Port.kMXP); /* old */
            gyro = new Pigeon2(Constants.Gyro.gyroId);
            drive = new SwerveDrive(Constants.SwerveDrive.configs);
            controls = new Controls();
            pathplanner = new FridoPathplanner(drive);
            liftingTower = new LiftingTowerSubsystem();

            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto", autoChooser);
            SmartDashboard.putData(liftingTower);
            SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());
    }

    public static synchronized Rotation2d getGyroRotation2d() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        //double angle = Utils.normalizeAngleRad(inverted * RobotContainer.gyro.getAngle() * Math.PI / 180.0);
        double angle = Math.IEEEremainder(inverted * gyro.getYaw().getValueAsDouble(), 360);
        return Rotation2d.fromDegrees(angle);
    }
    
    public static Command getAutoCommand(){
        return autoChooser.getSelected();
    }
}
