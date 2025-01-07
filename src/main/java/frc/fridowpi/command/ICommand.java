package frc.fridowpi.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ICommand {
    void requires(Subsystem... requirements);
}
