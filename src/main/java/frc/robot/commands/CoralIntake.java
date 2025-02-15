package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralIntake extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;

    public CoralIntake(){
        this.coralDispenserSubsystem = null;
        if (coralDispenserSubsystem != null)
        addRequirements(coralDispenserSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coralDispenserSubsystem.setMotorTopSpeed(Constants.CoralDispenser.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        coralDispenserSubsystem.stopMotorTop();
    }

    @Override
    public boolean isFinished() {
        return coralDispenserSubsystem.isForwardLimitSwitchPressedMotorTop();
    }
    
}
