package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.LiftingTowerSubsystem;
import frc.robot.swerve.FridoPathplanner;
import frc.robot.swerve.SwerveDrive;
import frc.robot.subsystems.ClimberSubsytem;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    // public static final AHRS gyroNavx; 
    public static final Pigeon2 gyro;
    public static final FridoPathplanner pathplanner;

    //public static final ClimberSubsytem climber = new ClimberSubsytem();
    //public static final CoralDispenserSubsystem coralDispenser = new CoralDispenserSubsystem();
    //public static final LiftingTowerSubsystem liftingTower = new LiftingTowerSubsystem();

    static {
        // gyroNavx = new AHRS(Port.kMXP); /* old */
        gyro = new Pigeon2(Constants.Gyro.gyroId);
        drive = new SwerveDrive(Constants.SwerveDrive.configs);
        controls = new Controls();
        pathplanner = new FridoPathplanner(drive);
    }

    // old, don't use
    /* 
    public static synchronized Rotation2d getGyroRotation2dFromNavx() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        //double angle = Utils.normalizeAngleRad(inverted * RobotContainer.gyro.getAngle() * Math.PI / 180.0);
        double angle = Math.IEEEremainder(inverted * gyroNavx.getAngle(), 360);
        return Rotation2d.fromDegrees(angle);
    }
    */

    public static synchronized Rotation2d getGyroRotation2d() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        //double angle = Utils.normalizeAngleRad(inverted * RobotContainer.gyro.getAngle() * Math.PI / 180.0);
        double angle = Math.IEEEremainder(inverted * gyro.getYaw().getValueAsDouble(), 360);
        return Rotation2d.fromDegrees(angle);
    }
    
    public Command getAutoCommand(){
        
        return pathplanner.getAutoCommandGroup(Constants.Autonomous.autoGroup);
    }
}
