package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CoralHeightPitchCommandGroup extends ParallelCommandGroup {
    public CoralHeightPitchCommandGroup(int state) {
        addCommands(new CoralGoToHeightState(Constants.parameters[state].height), new CoralGoToPitchState(Constants.parameters[state].pitchAngle), new InstantCommand(() -> {
            RobotContainer.leds.synchronizeLEDsWithStates();
        }));
    }

}