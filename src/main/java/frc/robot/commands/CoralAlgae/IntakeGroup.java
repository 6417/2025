package frc.robot.commands.CoralAlgae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.LiftingTower.CoralGoToPitchState;
import frc.robot.Constants;

public class IntakeGroup extends SequentialCommandGroup {
    public IntakeGroup() {
        addCommands(new CoralIntake(RobotContainer.coralDispenser), 
            new CoralGoToPitchState(RobotContainer.coralDispenser, Constants.CoralDispenser.steadyStateSetpoint),
            new InstantCommand(()-> RobotContainer.leds.coralLEDs()));
    }
}