package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AlgaeInCommandGroup extends SequentialCommandGroup {
    public AlgaeInCommandGroup() {
        addCommands(
            new AlgaeIntake(), new WaitCommand(Constants.CoralDispenser.waitAfterAlgaeIntake),
            new InstantCommand(() -> {
                // RobotContainer.coralDispenser.stopMotorTop();
            })
        );
    }

}