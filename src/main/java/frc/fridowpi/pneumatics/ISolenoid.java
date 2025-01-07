package frc.fridowpi.pneumatics;

import frc.fridowpi.initializer.Initialisable;
import edu.wpi.first.util.sendable.Sendable;

public interface ISolenoid extends AutoCloseable, Sendable, Initialisable {
    void set(boolean direction);

    int getChannel();

    void toggle();

    boolean get();
}
