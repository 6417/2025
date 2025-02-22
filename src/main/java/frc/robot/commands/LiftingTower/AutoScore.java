package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CoralAlgae.CoralAlgaeOutCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(int state) {
        addCommands(new CoralHeightPitchCommandGroup(state), 
                    new CoralAlgaeOutCommandGroup().withTimeout(0.5),
                    new CoralHeightPitchCommandGroup(1)
                    //,new CoralGoToPitchState(RobotContainer.coralDispenser, Constants.CoralDispenser.pitchUp)
        );
    }
}
