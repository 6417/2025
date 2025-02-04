package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralOuttake extends Command {
    private final CoralDispenserSubsystem coralSubsystem;

    public CoralOuttake() {
        this.coralSubsystem = RobotContainer.coralDispenser;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        coralSubsystem.setMotorTopSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stopMotorTop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}