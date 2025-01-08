package frc.fridowpi.joystick.joysticks;

import java.util.Optional;

import frc.fridowpi.joystick.IJoystickButtonId;

public enum POV implements IJoystickButtonId {
    DPadUp(100),
    DPadUpRight(101),
    DPadRight(102),
    DPadDownRight(103),
    DPadDown(104),
    DPadDownLeft(105),
    DPadLeft(106),
    DPadUpLeft(107),
    Lt(200),
    Rt(201);

    private final int buttonId;

    private POV(int id) {
        buttonId = id;
    }

    @Override
    public int getButtonId() {
        return buttonId;
    }
}
