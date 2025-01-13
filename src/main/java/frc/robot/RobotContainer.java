package frc.robot;

import java.util.List;

import frc.robot.swerve.SwerveDrive;
import frc.robot.Constants;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;

    static {
        drive = new SwerveDrive(Constants.SwerveDrive.configs);
        controls = new Controls();
    }
}
