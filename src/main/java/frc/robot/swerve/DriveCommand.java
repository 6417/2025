package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveCommand extends Command {
    public DriveCommand()
    {
        addRequirements(RobotContainer.drive);
    }
}
