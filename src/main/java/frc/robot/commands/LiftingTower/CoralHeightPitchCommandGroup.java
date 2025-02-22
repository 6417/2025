package frc.robot.commands.LiftingTower;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CoralHeightPitchCommandGroup extends ParallelCommandGroup {
    public CoralHeightPitchCommandGroup(int state) {
        addCommands(            
            new CoralGoToHeightState(RobotContainer.liftingTower, Constants.parameters[state].height),
            new CoralGoToPitchState(RobotContainer.coralDispenser, Constants.parameters[state].pitchAngle)
        );
    }
}