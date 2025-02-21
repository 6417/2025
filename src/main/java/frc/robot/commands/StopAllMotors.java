package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LiftingTowerSubsystem;

import frc.robot.RobotContainer;

public class StopAllMotors extends Command {
    public StopAllMotors(){
        addRequirements(RobotContainer.coralDispenser, RobotContainer.liftingTower);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        RobotContainer.coralDispenser.setMotorTopSpeed(0);
        RobotContainer.liftingTower.setMotorSpeed(0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
