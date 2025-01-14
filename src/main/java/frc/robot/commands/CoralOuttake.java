package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralOuttake extends Command {
    private final CoralDispenserSubsystem coralSubsystem;

    public CoralOuttake(CoralDispenserSubsystem subsystem) {
        this.coralSubsystem = subsystem;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        coralSubsystem.setMotorSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}