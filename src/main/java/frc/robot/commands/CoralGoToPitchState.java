package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralGoToPitchState extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;
    private final Rotation2d angle;

    CoralGoToPitchState(CoralDispenserSubsystem subsystem, Rotation2d angle){
        this.coralDispenserSubsystem = subsystem;
        if (coralDispenserSubsystem != null)
            addRequirements(coralDispenserSubsystem);
        this.angle = angle;

    }

    @Override
    public void initialize() {
        coralDispenserSubsystem.setPitch(angle.getDegrees());
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
