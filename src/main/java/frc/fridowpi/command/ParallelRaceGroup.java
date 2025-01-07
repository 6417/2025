package frc.fridowpi.command;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class ParallelRaceGroup extends edu.wpi.first.wpilibj2.command.ParallelRaceGroup implements ICommand {
    private FridoCommand commandProxy;

    public ParallelRaceGroup(edu.wpi.first.wpilibj2.command.Command... commands) {
        super(commands);
        commandProxy = new FridoCommand();
    }

    @Override
    public void requires(Subsystem... requirements) {
        commandProxy.requires(requirements);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return commandProxy.getRequirements();
    }
}
