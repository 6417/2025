package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LiftingTowerSubsystem;

public class CoralGoToHeightState extends Command {
    private final LiftingTowerSubsystem liftingTowerSubsystem;
    private final double height;

    CoralGoToHeightState(LiftingTowerSubsystem subsystem, double height){
        this.liftingTowerSubsystem = subsystem;
        /*if (liftingTowerSubsystem != null)
            addRequirements(liftingTowerSubsystem);*/
        this.height = height;

    }

    @Override
    public void initialize() {
        liftingTowerSubsystem.setHeight(height);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        liftingTowerSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return liftingTowerSubsystem.isAtDesiredHeight();
    }
    
}
