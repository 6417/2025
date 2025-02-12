package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LiftingTowerSubsystem;

public class CoralGoToHeightState extends Command {
    private final LiftingTowerSubsystem LiftingTowerSubsystem;
    private final double height;

    CoralGoToHeightState(double height){
        this.LiftingTowerSubsystem = null;//RobotContainer.liftingTower;
        if (LiftingTowerSubsystem != null)
        addRequirements(LiftingTowerSubsystem);
        this.height = height;

    }

    @Override
    public void initialize() {
        LiftingTowerSubsystem.setHeight(height);
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
