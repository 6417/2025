package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CoralAlgae.CoralAlgaeOutCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class FinishCommand extends SequentialCommandGroup {
    public FinishCommand() {
        addCommands(new CoralAlgaeOutCommandGroup().withTimeout(0.5),
                    new CoralHeightPitchCommandGroup(Constants.CoralDispenser.steadyState),
                    new InstantCommand(()-> RobotContainer.leds.normalLeds())
        );
    }
}
