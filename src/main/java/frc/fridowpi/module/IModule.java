package frc.fridowpi.module;

import frc.fridowpi.initializer.Initialisable;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Collection;

public interface IModule extends Subsystem, Initialisable {
    Collection<IModule> getAllSubModules();
    Collection<IModule> getSubModules();
    void registerSubmodule(IModule... subModule);
}

