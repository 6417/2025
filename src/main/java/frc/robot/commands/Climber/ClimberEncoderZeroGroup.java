package frc.robot.commands.Climber;

import java.security.cert.LDAPCertStoreParameters;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LiftingTower.CoralGoToHeightState;
import frc.robot.commands.LiftingTower.CoralGoToPitchState;
import frc.robot.commands.LiftingTower.CoralHeightPitchCommandGroup;

public class ClimberEncoderZeroGroup extends ParallelCommandGroup {
    public ClimberEncoderZeroGroup() {
        addCommands(new ClimberEncoderZero(RobotContainer.climber),            
            new CoralGoToPitchState(RobotContainer.coralDispenser, Constants.CoralDispenser.pitchDown),
            new CoralGoToHeightState(RobotContainer.liftingTower, 1.0));
    }
}