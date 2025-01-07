package frc.fridowpi.joystick.joysticks;

import frc.fridowpi.joystick.IJoystickButtonId;

public enum LogitechExtreme implements IJoystickButtonId {
    _1(1),
    _2(2),
    _3(3),
    _4(4),
    _5(5),
    _6(6),
    _7(7),
    _8(8),
    _9(9),
    _10(10),
    _11(11),
    _12(12);

    private final int buttonId;

    private LogitechExtreme(int id) {
        buttonId = id;
    }

    @Override
    public int getButtonId() {
        return buttonId;
    }
}
