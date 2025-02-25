package frc.robot.commands.CoralAlgae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CoralDispenser;
import frc.robot.Controls.IntakeState;
import frc.robot.commands.LiftingTower.CoralHeightPitchCommandGroup;

public class AlgaeOuttakeCommandGroup extends SequentialCommandGroup {
    public AlgaeOuttakeCommandGroup(int state) {
        addCommands(
                new CoralAlgaeOuttake(RobotContainer.coralDispenser),
                new WaitCommand(Constants.CoralDispenser.waitAfterOuttake),
                new InstantCommand(() -> {
                    RobotContainer.coralDispenser.stopMotorTop();
                }));
    }

}