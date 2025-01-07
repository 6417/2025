package frc.fridowpi.command;

import frc.fridowpi.module.IModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public class FridoCommand extends Command implements ICommand {
    @Override
    public void requires(Subsystem... requirements) {
        Set<Subsystem> allRequirements = new HashSet<>(Set.of(requirements));

        addRequirements(requirements);

		// Add submodule requirements
        Set<Subsystem> subModuleRequirements = Arrays.stream(requirements)
                .filter((req) -> req instanceof IModule)
                .map((mod) -> (IModule) mod)
                .map(IModule::getAllSubModules)
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());
        allRequirements.addAll(subModuleRequirements);
        addRequirements(allRequirements.toArray(Subsystem[]::new));
    }
}
