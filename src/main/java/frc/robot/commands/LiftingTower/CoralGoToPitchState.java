package frc.robot.commands.LiftingTower;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralGoToPitchState extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;
    private final double angle;

    CoralGoToPitchState(CoralDispenserSubsystem subsystem, double angle){
        this.coralDispenserSubsystem = subsystem; //subsystem;
        if (coralDispenserSubsystem != null)
            addRequirements(coralDispenserSubsystem);
        this.angle = angle;

    }

    @Override
    public void initialize() {
        coralDispenserSubsystem.setPitch(angle);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return coralDispenserSubsystem.isAtDesiredPitch();
    }
    
}
