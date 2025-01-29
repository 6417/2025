package frc.robot.swerve;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.ColorSensorV3.Register;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.List;
import java.util.Map;
import java.util.function.Function;
import edu.wpi.first.math.Pair;
public class FridoPathplanner {
    private SwerveDrive drive;
    
    public FridoPathplanner(SwerveDrive drive) {
        this.drive = drive;

        // configuration
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            config = null;
        }

        AutoBuilder.configure(
                drive::getPose,
                drive::resetOdoemetry,
                drive::getChassisSpeeds,
                drive::setChassisSpeeds,
                new PPHolonomicDriveController(
                        new PIDConstants(1, 0.0, 0.2),
                        new PIDConstants(1.5, 0.0, 0.05)),
                config,
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive
        );

    }

    public void registerCommand(String name, Command command) {
        NamedCommands.registerCommand(name, command);
    }

    public Command getAutoCommandGroup(String fileName) {
        try {
           return new PathPlannerAuto(fileName);

        } catch (Exception e) {
            DriverStation.reportError("PathPlanner failed: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    // only to test; don't use
    public Command getAutonomousCommand(String fileName) {

        // return new PathPlannerAuto("Example Auto");
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(fileName);

            // Possible to implement a Path from here
    
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner failed: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}

