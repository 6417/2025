package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class manualClimberControl extends Command {
    private final frc.robot.subsystems.ClimberSubsystem climber;
    private final Joystick joystick;

    public manualClimberControl(frc.robot.subsystems.ClimberSubsystem climber, Joystick joystick) {
        this.climber = climber;
        this.joystick = joystick;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setSpeed(joystick.getRawAxis(1));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        climber.setSpeed(0);
    }
    
}
