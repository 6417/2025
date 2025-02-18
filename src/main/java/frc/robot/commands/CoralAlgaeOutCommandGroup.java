package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Controls.IntakeState;

public class CoralAlgaeOutCommandGroup extends SequentialCommandGroup {
    public CoralAlgaeOutCommandGroup() {
        addCommands(
            new CoralAlgaeOuttake(RobotContainer.coralDispenser), new WaitCommand(Constants.CoralDispenser.waitAfterOuttake),
            new InstantCommand(() -> {
                RobotContainer.controls.activeIntakeState = IntakeState.OUTTAKE;
            }),
            new InstantCommand(() -> {
                RobotContainer.coralDispenser.stopMotorTop();
            })
        );
    }

}