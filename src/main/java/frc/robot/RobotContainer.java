package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    public static final AHRS gyro;

    static {
        gyro = new AHRS(Port.kMXP);
        drive = new SwerveDrive(Constants.SwerveDrive.configs);
        controls = new Controls();
    }
}
