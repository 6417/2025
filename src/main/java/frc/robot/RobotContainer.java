package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.FridoPathplanner;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    public static final AHRS gyro;
    public static final Pigeon2 gyroPigeon;
    public static final FridoPathplanner pathplanner;

    static {
        gyro = new AHRS(Port.kMXP);
        gyroPigeon = new Pigeon2(0);;
        drive = new SwerveDrive(Constants.SwerveDrive.configs);
        controls = new Controls();
        pathplanner = new FridoPathplanner(drive);
    }

    public static synchronized Rotation2d getGyroRotation2d() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        //double angle = Utils.normalizeAngleRad(inverted * RobotContainer.gyro.getAngle() * Math.PI / 180.0);
        double angle = Math.IEEEremainder(inverted * gyro.getAngle(), 360);
        return Rotation2d.fromDegrees(angle);
    }
    public static synchronized Rotation2d getGyroRotation2dFromPigeon2() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        //double angle = Utils.normalizeAngleRad(inverted * RobotContainer.gyro.getAngle() * Math.PI / 180.0);
        double angle = Math.IEEEremainder(inverted * gyroPigeon.getYaw().getValueAsDouble(), 360);
        return Rotation2d.fromDegrees(angle);
    }
    
    public Command getAutoCommand(){
        
        return pathplanner.getAutoCommandGroup("AutoPathLeftStart");
    }
}
