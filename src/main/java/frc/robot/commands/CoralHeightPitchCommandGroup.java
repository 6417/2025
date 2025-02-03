package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

public class CoralHeightPitchCommandGroup extends ParallelCommandGroup {
    public CoralHeightPitchCommandGroup(int state) {
        addCommands(new CoralGoToHeightState(Constants.parameters[state].height), new CoralGoToPitchState(Constants.parameters[state].pitchAngle));
    }

}