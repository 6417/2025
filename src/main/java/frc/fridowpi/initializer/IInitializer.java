package frc.fridowpi.initializer;

public interface IInitializer extends Initialisable {
    InitialisableComposer compose(Initialisable initialisables);

    void addInitialisable(Initialisable initialisable);

    void removeInitialisable(Initialisable... initialisables);

    InitialisableComposer after(Initialisable initialisable, Initialisable after);

    boolean willBeInitialized(Initialisable ini);

    void addComposer(InitialisableComposer initialisableComposer);
}
