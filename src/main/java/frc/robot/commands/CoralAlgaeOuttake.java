package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralAlgaeOuttake extends Command {
    private final CoralDispenserSubsystem coralSubsystem;

    public CoralAlgaeOuttake() {
        this.coralSubsystem = null;//RobotContainer.coralDispenser;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        coralSubsystem.setMotorTopSpeed(Constants.CoralDispenser.outtakeSpeed);
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