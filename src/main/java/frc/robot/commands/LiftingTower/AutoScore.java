package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralAlgae.CoralAlgaeOutCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(int state) {
        addCommands(new InstantCommand(()->RobotContainer.leds.synchronizeLEDsWithStates()),
            new CoralHeightPitchCommandGroup(state), 
                    new CoralAlgaeOutCommandGroup().withTimeout(0.5),
                    new CoralHeightPitchCommandGroup(Constants.CoralDispenser.steadyState),
                    new InstantCommand(()-> RobotContainer.leds.normalLeds())
                    //,new CoralGoToPitchState(RobotContainer.coralDispenser, Constants.CoralDispenser.pitchUp)
        );
    }
}
