package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class ZeroCoralPitch extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;

    private ZeroCoralPitch(CoralDispenserSubsystem subsystem){
        this.coralDispenserSubsystem = subsystem;
        addRequirements(coralDispenserSubsystem);
    }

    @Override
    public void initialize() {
        coralDispenserSubsystem.resetPitchEncoder();
        coralDispenserSubsystem.setPitch(Constants.CoralDispenser.zeroingPosition);
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
