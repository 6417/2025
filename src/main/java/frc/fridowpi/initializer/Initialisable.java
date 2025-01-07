package frc.fridowpi.initializer;

public interface Initialisable {
    void init();

    boolean isInitialized();

    default void requires(Initialisable initialisable) {
        Initializer.getInstance().after(initialisable, this);
    }
}
