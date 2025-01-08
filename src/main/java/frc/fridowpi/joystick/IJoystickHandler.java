package frc.fridowpi.joystick;

import java.util.List;
import java.util.function.Function;

public interface IJoystickHandler {
    void init();

    void bindAll(List<Binding> bindings);

    void bind(Binding binding);

    void bind(JoystickBindable bindable);

    void setupJoysticks(List<IJoystickId> joystickIds);

    public IJoystick getJoystick(IJoystickId id);

    void setJoystickFactory(Function<IJoystickId, IJoystick> factory);
}
