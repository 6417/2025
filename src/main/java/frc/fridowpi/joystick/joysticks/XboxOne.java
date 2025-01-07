package frc.fridowpi.joystick.joysticks;

import frc.fridowpi.joystick.IJoystickButtonId;

public enum XboxOne implements IJoystickButtonId {
    a(1),
    b(2),
    x(3),
    y(4),
    lb(5),
    rb(6),
    back(7),
    start(8),
    leftJoystick(9),
    rightJoystick(10),
    lt(11),
    rt(12);

    private final int buttonId;

    private XboxOne(int id) {
        buttonId = id;
    }

    @Override
    public int getButtonId() {
        return buttonId;
    }
}
