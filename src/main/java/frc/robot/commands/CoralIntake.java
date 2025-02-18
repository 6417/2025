package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Controls.IntakeState;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralIntake extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;

    public CoralIntake(CoralDispenserSubsystem subsystem){
        this.coralDispenserSubsystem = subsystem;
        if (coralDispenserSubsystem != null)
            addRequirements(coralDispenserSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.controls.activeIntakeState = IntakeState.INTAKE;
    }

    @Override
    public void execute() {
        coralDispenserSubsystem.setMotorTopSpeed(Constants.CoralDispenser.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        coralDispenserSubsystem.stopMotorTop();
    }

    @Override
    public boolean isFinished() {
        return coralDispenserSubsystem.isForwardLimitSwitchPressedMotorTop();
    }
    
}
