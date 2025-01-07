package frc.fridowpi.module;

import frc.fridowpi.initializer.Initializer;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.JoystickBindable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.*;
import java.util.stream.Collectors;

public class Module extends SubsystemBase implements JoystickBindable, IModule {
    private Set<IModule> submodules = new HashSet<>();

    public Module() {
        Initializer.getInstance().addInitialisable(this);
    }

    @Override
    public void registerSubmodule(IModule... modules) {
        Arrays.stream(modules).forEach((module) -> {
            assert (getAllSubModules().stream().noneMatch((other) -> this == other) && module != this) : "'this' can not be a submodule of its self";
        });
        submodules.addAll(Set.of(modules));
    }

    @Override
    public Collection<IModule> getSubModules() {
        return Collections.unmodifiableSet(submodules);
    }

    @Override
    public Collection<IModule> getAllSubModules() {
        Set<IModule> result = submodules.stream()
                .filter((module) -> {
                    assert module != this : "'this' can not be a submodule of its self";
                    return module.getSubModules().size() > 0;
                })
                .map(IModule::getAllSubModules)
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());
        result.addAll(submodules);
        return result;
    }

    private boolean initialized = false;

    @Override
    public void init() {
        initialized = true;
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public List<Binding> getMappings() {
        return new ArrayList<>();
    }
}