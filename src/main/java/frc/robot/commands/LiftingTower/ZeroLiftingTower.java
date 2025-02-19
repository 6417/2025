package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftingTowerSubsystem;

public class ZeroLiftingTower extends Command {
    private final LiftingTowerSubsystem liftingTowerSubsystem;

    private ZeroLiftingTower(LiftingTowerSubsystem liftingTowerSubsystem){
        this.liftingTowerSubsystem = liftingTowerSubsystem;
        addRequirements(liftingTowerSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        liftingTowerSubsystem.setPercent(Constants.LiftingTower.zeroingSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        liftingTowerSubsystem.stopMotors();
        liftingTowerSubsystem.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return liftingTowerSubsystem.isBottomSwitchPressed();
    }
}
