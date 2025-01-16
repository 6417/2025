package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class ZeroCoralPitch extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;

    private ZeroCoralPitch(CoralDispenserSubsystem coralDispenserSubsystem){
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        addRequirements(coralDispenserSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coralDispenserSubsystem.setPitchPercent(Constants.CoralDispenser.zeroingSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        coralDispenserSubsystem.stopMotorPitch();
        coralDispenserSubsystem.resetPitchEncoder();
    }

    @Override
    public boolean isFinished() {
        return coralDispenserSubsystem.isReverseLimitSwitchPressed();
    }
    
}
