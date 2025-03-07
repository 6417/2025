package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(int state) {
        addCommands(new CoralHeightPitchCommandGroup(state));
    }
}
