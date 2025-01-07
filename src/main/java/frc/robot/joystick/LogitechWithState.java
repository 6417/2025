
package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.joystick.WPIJoystick;
import frc.fridowpi.joystick.joysticks.POV;
import frc.robot.Constants;

/**
 * LogitechWithState
 */
public class LogitechWithState extends WPIJoystick {

	public LogitechWithState(IJoystickId port) {
		super(port);
	}

    @Override
    public Trigger getButton(IJoystickButtonId id) {
        if (id instanceof POV && id.getButtonId() >= 100 && id.getButtonId() < 200) {
            return new Trigger(() -> isPressedPOV((POV) id));
        }
        return new JoystickButton(this, id.getButtonId());

    }

    private boolean isPressedPOV(POV id) {
        return id.pov.get().getDegrees() == getPOV();
    }
}
