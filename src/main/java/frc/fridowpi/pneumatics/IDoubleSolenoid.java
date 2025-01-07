package frc.fridowpi.pneumatics;

import frc.fridowpi.initializer.Initialisable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IDoubleSolenoid extends AutoCloseable, Sendable, Initialisable {
    void set(final Value value);

    Value get();

    void toggle();

    int getFwdChannel();

    int getRevChannel();

    boolean isFwdSolenoidDisabled();

    boolean isRevSolenoidDisabled();
}