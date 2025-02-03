package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftingTowerSubsystem;

public class ZeroLiftingTower extends Command {
    private final LiftingTowerSubsystem LiftingTowerSubsystem;

    private ZeroLiftingTower(LiftingTowerSubsystem LiftingTowerSubsystem){
        this.LiftingTowerSubsystem = LiftingTowerSubsystem;
        addRequirements(LiftingTowerSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        LiftingTowerSubsystem.setPercent(Constants.LiftingTower.zeroingSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        LiftingTowerSubsystem.stopMotors();
        LiftingTowerSubsystem.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return LiftingTowerSubsystem.isForwardLimitSwitchPressed();
    }
    
}
