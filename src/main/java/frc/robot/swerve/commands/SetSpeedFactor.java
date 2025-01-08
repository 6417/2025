package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Controls.DriveSpeed;

public class SetSpeedFactor extends Command {
    private DriveSpeed speedFactor;
    private static SubsystemBase speedFactorCommandRequirement = new SubsystemBase() {
    };

    public SetSpeedFactor(DriveSpeed speedFactor) {
        this.speedFactor = speedFactor;
        addRequirements(speedFactorCommandRequirement);
    }

    @Override
    public void initialize() {
        RobotContainer.controls.setActiveSpeedFactor(speedFactor);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
