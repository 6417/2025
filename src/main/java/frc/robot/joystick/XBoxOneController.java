package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.WPIJoystick;
import frc.fridowpi.joystick.joysticks.POV;
import frc.robot.Constants;

public class XBoxOneController extends WPIJoystick {

	public XBoxOneController(IJoystickId port) {
		super(port);
		setThrottleChannel(5);
		setTwistChannel(4);
	}


    public double getLtValue() {
        return getRawAxis(2);
    }

    public double getRtValue() {
        return getRawAxis(3);
    }

    @Override
    public Trigger getButton(IJoystickButtonId id) {
        if (id instanceof POV && id.getButtonId() >= 100 && id.getButtonId() < 200) {
            return new Trigger(() -> isPressedPOV((POV) id));
        } else if (id instanceof POV && id.getButtonId() >= 200 && id.getButtonId() < 300) {
            return new Trigger(() -> isPressedLorRT((POV) id));
        } else if (id instanceof IdsWithState && id.getButtonId() >= Constants.Joystick.idCounterStart) {
			return ((IdsWithState) id).toButtonOnJoystick(this);
		}
        return new JoystickButton(this, id.getButtonId());
    }

    private boolean isPressedPOV(POV id) {
        return id.pov.get().getDegrees() == getPOV();
    }

    private boolean isPressedLorRT(POV id) {
        if (id == POV.Lt) {
            return getLtValue() >= Constants.Joystick.lt_rt_reshold;
        }
        return getRtValue() >= Constants.Joystick.lt_rt_reshold;
    }
}
