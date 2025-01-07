package frc.fridowpi.joystick;

import frc.fridowpi.initializer.Initialisable;

import java.util.List;
import java.util.function.Function;

public interface IJoystickHandler extends Initialisable {
    void bindAll(List<Binding> bindings);

    void bind(Binding binding);

    void bind(JoystickBindable bindable);

    void setupJoysticks(List<IJoystickId> joystickIds);

    public IJoystick getJoystick(IJoystickId id);

    void setJoystickFactory(Function<IJoystickId, IJoystick> factory);
}
