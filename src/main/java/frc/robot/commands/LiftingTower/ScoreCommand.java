package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(int state) {
        addCommands(new InstantCommand(()->RobotContainer.leds.synchronizeLEDsWithStates()),
            new CoralHeightPitchCommandGroup(state));
    }
}
