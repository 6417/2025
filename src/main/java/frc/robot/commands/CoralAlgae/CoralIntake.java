package frc.robot.commands.CoralAlgae;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Controls.IntakeState;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralIntake extends Command {
    private final CoralDispenserSubsystem coralDispenserSubsystem;
    private final Debouncer debouncer;

    public CoralIntake(CoralDispenserSubsystem subsystem){
        this.coralDispenserSubsystem = subsystem;
        if (coralDispenserSubsystem != null)
            addRequirements(coralDispenserSubsystem);
        debouncer = new Debouncer(0.08, DebounceType.kBoth);
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
        return debouncer.calculate(coralDispenserSubsystem.isForwardLimitSwitchPressedMotorTop());
    }
    
}
