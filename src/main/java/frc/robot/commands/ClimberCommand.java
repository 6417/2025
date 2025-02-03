package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsytem;

public class ClimberCommand extends Command {
    private final ClimberSubsytem ClimberSubsytem;
    private final double position;

    ClimberCommand(double position) {
        this.ClimberSubsytem = RobotContainer.climber;
        addRequirements(ClimberSubsytem);
        this.position = position;

    }

    @Override
    public void initialize() {
        ClimberSubsytem.setPosition(position);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
