package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LiftingTowerSubsystem;

public class TowerManualControl extends Command {
    private LiftingTowerSubsystem tower = null;

    public TowerManualControl(LiftingTowerSubsystem tower) {
        this.tower = tower;

        if (tower != null) {
            addRequirements(tower);
        }
    }

    private static double applyDeadband(double x, double deadBand) {
        return Math.abs(x) < deadBand ? 0 : (Math.abs(x) - deadBand) / (1 - deadBand) * Math.signum(x);
    }

    @Override
    public void execute() {
        final double deadZone = 0.1;
        double input = RobotContainer.controls.operatorJoystick.getLeftY();
        input = applyDeadband(input, deadZone);

        System.out.println(input);
        tower.setPercent(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
