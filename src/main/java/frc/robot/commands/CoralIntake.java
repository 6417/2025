package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralIntake extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;

    private CoralIntake(CoralDispenserSubsystem coralDispenserSubsystem){
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        addRequirements(coralDispenserSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coralDispenserSubsystem.setMotorSpeed(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        coralDispenserSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
