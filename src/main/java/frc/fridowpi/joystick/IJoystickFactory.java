package frc.fridowpi.joystick;

public interface IJoystickFactory {
    IJoystick create(IJoystickId id);
}
