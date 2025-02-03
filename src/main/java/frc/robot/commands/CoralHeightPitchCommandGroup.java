package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralHeightPitchCommandGroup extends ParallelCommandGroup {
    public CoralHeightPitchCommandGroup(int state) {
        addCommands(new CoralGoToHeightState(Constants.parameters[state].height), new CoralGoToPitchState(Constants.parameters[state].pitchAngle));
    }

}