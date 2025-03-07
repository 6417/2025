package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LiftingTowerSubsystem;

public class CoralGoToHeightState extends Command {
    private final LiftingTowerSubsystem liftingTowerSubsystem;
    private final double height;

    public CoralGoToHeightState(LiftingTowerSubsystem subsystem, double height){
        this.liftingTowerSubsystem = subsystem;
        assert height >= 1 : "[CoralGoToHeightState] height below soft limit requested";
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

    }

    @Override
    public boolean isFinished() {
        return liftingTowerSubsystem.isAtDesiredHeight();
    }
}
