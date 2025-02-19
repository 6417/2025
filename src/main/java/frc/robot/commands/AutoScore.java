package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(int state) {
        addCommands(new CoralHeightPitchCommandGroup(state), 
                    new CoralAlgaeOutCommandGroup().withTimeout(0.5),
                    new CoralHeightPitchCommandGroup(0)
        );
    }
}
