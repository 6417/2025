package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.WPIJoystick;
import frc.robot.Constants;

public class XBox360WithState extends WPIJoystick {


    public XBox360WithState(IJoystickId port) {
        super(port);
        setXChannel(0);
        setYChannel(1);
        setThrottleChannel(2);
        setTwistChannel(3);
        setZChannel(4);
    }

    @Override
    public Trigger getButton(IJoystickButtonId id) {
        if (id.getButtonId() >= Constants.Joystick.idCounterStart) {
            var idNew = (IdsWithState) id;
            return idNew.toButtonOnJoystick(this);
        }
        return super.getButton(id);
    }
}
