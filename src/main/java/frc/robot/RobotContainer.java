package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    public static final AHRS gyro;

    static {
        gyro = new AHRS(Port.kMXP);
        drive = new SwerveDrive(Constants.SwerveDrive.configs);
        controls = new Controls();
    }

    public static Rotation2d getGyroRotation2d() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        double angle = Utils.normalizeAngleRad(inverted * RobotContainer.gyro.getAngle() * Math.PI / 180.0);
        return Rotation2d.fromRadians(angle);
    }

    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("Example Auto");
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

            // Possible to implement a Path from here

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner failed: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
