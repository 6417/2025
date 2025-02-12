package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Controls.Climberstate;

public class ClimberCommand extends Command {
    private final ClimberSubsystem climber;
    private final double position;
    private final Climberstate state;

    public ClimberCommand(double position, Climberstate state) {
        this.climber = null;//RobotContainer.climber;
        addRequirements(climber);
        this.position = position;
        this.state = state;
    }

    @Override
    public void initialize() {
        switch (state) {
            case kForward:
                climber.setPositionForward(position);
                break;
            
            case kBack:
                climber.setPositionUnderLoad(position);
                break;

            case kSteady:
                climber.setPositionForward(position);
            default:
                break;
        }
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
