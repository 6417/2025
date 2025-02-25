package frc.robot.commands.LiftingTower;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralGoToPitchState extends Command{
    private final CoralDispenserSubsystem coralDispenserSubsystem;
    private final double angle;
    private double newPitchOffset=0;

    public CoralGoToPitchState(CoralDispenserSubsystem subsystem, double angle){
        this.coralDispenserSubsystem = subsystem; //subsystem;
        if (coralDispenserSubsystem != null)
            addRequirements(coralDispenserSubsystem);
            
        assert Constants.CoralDispenser.pitchMotorReverseLimit <= angle;
        assert Constants.CoralDispenser.pitchMotorForwardLimit >= angle;


        this.angle = angle;

    }

    @Override
    public void initialize() {

        newPitchOffset -= RobotContainer.controls.dispenserOffset;

        coralDispenserSubsystem.setPitch(angle+newPitchOffset);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return coralDispenserSubsystem.isAtDesiredPitch();
    }
    
}
