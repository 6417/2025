package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CoralAlgaeOutCommandGroup extends SequentialCommandGroup {
    public CoralAlgaeOutCommandGroup() {
        addCommands(
            new CoralAlgaeOuttake(), new WaitCommand(Constants.CoralDispenser.waitAfterOuttake),
            new InstantCommand(() -> {
                // RobotContainer.coralDispenser.stopMotorTop();
            })
        );
    }

}