package frc.robot.joystick;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.joystick.IJoystickButtonId;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.joystick.joysticks.POV;
import frc.fridowpi.joystick.joysticks.Xbox360;
import frc.fridowpi.joystick.joysticks.XboxOne;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.abstraction.baseClasses.BDrive.SpeedFactor;
import frc.robot.joystick.IdsWithState.State;

/**
 * JoystickBindings2024
 */
public class JoystickBindings2024 {
	private static JoystickBindings2024 instance = new JoystickBindings2024();
	public static ArrayList<Binding> tmp_bindings = new ArrayList<Binding>();

	public static JoystickBindings2024 getInstance() {
		return instance;
	}

	public static List<Binding> getBindingsSwerve2024() {
		tmp_bindings.clear();

		/// ---- Drive ---- ///

		if (Controls.getControlMode() == Controls.ControlMode.CONVENTIONAL) {
			quickBindToggle(POV.Lt,
					() -> {
						System.out.println("FAST mode");
						Controls.setActiveSpeedFactor(SpeedFactor.FAST);
					},
					() -> Controls.setActiveSpeedFactor(SpeedFactor.DEFAULT_SPEED));
			quickBindToggle(POV.Rt,
					() -> {
						System.out.println("SLOW mode");
						Controls.setActiveSpeedFactor(SpeedFactor.SLOW);
					},
					() -> Controls.setActiveSpeedFactor(SpeedFactor.DEFAULT_SPEED));
		};

		// quickBind(XboxOne.back, new InstantCommand(() -> {
		// 	FridoNavx.getInstance().reset();
		// 	RobotContainer.drive().resetOdometry();
		// 	System.out.println("<<<[ Zeroed ]>>>");
		// }));


		/// ---- Secondary Controller ---- ///

		// Switch States
		quickBindSecondary(XboxOne.start, new InstantCommand(() -> {
			Joystick2024.getInstance().setState(State.ENDGAME);
			System.out.println("<<<[ Endgame Activated ]>>>");
		}));

		return tmp_bindings;
	}

	public static List<Binding> getBindingsLogitechTest() {
		tmp_bindings.clear();
		quickBindWhileHeld(Logitech.a, () -> System.out.println("a"));
		quickBind(Logitech.b, () -> System.out.println("b"));
		quickBind(Logitech.x, () -> System.out.println("x"));
		quickBind(Logitech.y, () -> System.out.println("y"));
		quickBind(Logitech.start, () -> System.out.println("start"));
		quickBind(Logitech.back, () -> System.out.println("back"));
		quickBind(Logitech.lb, () -> System.out.println("lb"));
		quickBind(Logitech.rb, () -> System.out.println("rb"));
		quickBind(Logitech.lt, () -> System.out.println("lt"));
		quickBind(Logitech.rt, () -> System.out.println("rt"));

		quickBind(POV.DPadUp, () -> System.out.println("dpad up"));
		quickBind(POV.DPadUpRight, () -> System.out.println("dpad up right"));
		quickBind(POV.DPadRight, () -> System.out.println("dpad right"));
		quickBind(POV.DPadDownRight, () -> System.out.println("dpad down right"));
		quickBind(POV.DPadDown, () -> System.out.println("dpad down"));
		quickBind(POV.DPadDownLeft, () -> System.out.println("dpad down left"));
		quickBind(POV.DPadLeft, () -> System.out.println("dpad left"));
		quickBind(POV.DPadUpLeft, () -> System.out.println("dpad up left"));

		quickBind(POV.Lt, () -> System.out.println("POV lt"));
		quickBind(POV.Rt, () -> System.out.println("POV rt"));
		return tmp_bindings;
	}

	public static List<Binding> getBindingsXboxTest() {
		tmp_bindings.clear();
		quickBind(Xbox360.a, () -> System.out.println("a"));
		quickBind(Xbox360.b, () -> System.out.println("b"));
		quickBind(Xbox360.x, () -> System.out.println("x"));
		quickBind(Xbox360.y, () -> System.out.println("y"));
		quickBind(Xbox360.start, () -> System.out.println("start"));
		quickBind(Xbox360.back, () -> System.out.println("back"));
		quickBind(Xbox360.lb, () -> System.out.println("lb"));
		quickBind(Xbox360.rb, () -> System.out.println("rb"));
		quickBind(Xbox360.lt, () -> System.out.println("lt"));
		quickBind(Xbox360.rt, () -> System.out.println("rt"));

		quickBind(POV.DPadUp, () -> System.out.println("dpad up"));
		quickBind(POV.DPadUpRight, () -> System.out.println("dpad up right"));
		quickBind(POV.DPadRight, () -> System.out.println("dpad right"));
		quickBind(POV.DPadDownRight, () -> System.out.println("dpad down right"));
		quickBind(POV.DPadDown, () -> System.out.println("dpad down"));
		quickBind(POV.DPadDownLeft, () -> System.out.println("dpad down left"));
		quickBind(POV.DPadLeft, () -> System.out.println("dpad left"));
		quickBind(POV.DPadUpLeft, () -> System.out.println("dpad up left"));

		quickBind(POV.Lt, () -> System.out.println("POV lt"));
		quickBind(POV.Rt, () -> System.out.println("POV rt"));
		return tmp_bindings;
	}

	// On single press
	public static void quickBind(IJoystickButtonId button, State state, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, IdsWithState.create(button, state),
				Trigger::onTrue, new InstantCommand(fn)));
	}

	public static void quickBind(IJoystickButtonId button, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button,
				Trigger::onTrue, new InstantCommand(fn)));
	}

	public static void quickBind(IJoystickButtonId button, Command cmd) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::onTrue, cmd));
	}

	// While held
	public static void quickBindWhileHeld(IJoystickButtonId button, State state, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, IdsWithState.create(button, state),
				Trigger::whileTrue, new InstantCommand(fn)));
	}

	public static void quickBindWhileHeld(IJoystickButtonId button, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button,
				Trigger::whileTrue, new InstantCommand(fn)));
	}

	public static void quickBindWhileHeld(IJoystickButtonId button, Command cmd) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::whileTrue, cmd));
	}

	// Toggle
	public static void quickBindToggle(IJoystickButtonId button, Runnable on, Runnable off) {
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::onTrue,
				new InstantCommand(on)));
		tmp_bindings.add(new Binding(Constants.Joystick.primaryJoystickId, button, Trigger::onFalse,
				new InstantCommand(off)));
	}

	/// ----- For secondary joystick -----///
	// On single press
	public static void quickBindSecondary(IJoystickButtonId button, State state, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, IdsWithState.create(button, state),
				Trigger::onTrue, new InstantCommand(fn)));
	}

	public static void quickBindSecondary(IJoystickButtonId button, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, button,
				Trigger::onTrue, new InstantCommand(fn)));
	}

	public static void quickBindSecondary(IJoystickButtonId button, Command cmd) {
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, button, Trigger::onTrue, cmd));
	}

	// While held
	public static void quickBindWhileHeldSecondary(IJoystickButtonId button, State state, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, IdsWithState.create(button, state),
				Trigger::whileTrue, new InstantCommand(fn)));
	}

	public static void quickBindWhileHeldSecondary(IJoystickButtonId button, Runnable fn) {
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, button,
				Trigger::whileTrue, new InstantCommand(fn)));
	}

	public static void quickBindWhileHeldSecondary(IJoystickButtonId button, Command cmd) {
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, button, Trigger::whileTrue, cmd));
	}

	// Toggle
	public static void quickBindToggleSecondary(IJoystickButtonId button, Runnable on, Runnable off) {
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, button, Trigger::onTrue,
				new InstantCommand(on)));
		tmp_bindings.add(new Binding(Constants.Joystick.secondaryJoystickId, button, Trigger::onFalse,
				new InstantCommand(off)));
	}

	// If using different States
	public static void setState(State state) {
		IdsWithState.activeState = state;
	}

	public static List<Binding> getBindingsXboxOneTest() {
		tmp_bindings.clear();
		quickBind(XboxOne.a, () -> System.out.println("a"));
		quickBind(XboxOne.b, () -> System.out.println("b"));
		quickBind(XboxOne.x, () -> System.out.println("x"));
		quickBind(XboxOne.y, () -> System.out.println("y"));
		quickBind(XboxOne.start, () -> System.out.println("start"));
		quickBind(XboxOne.back, () -> System.out.println("back"));
		quickBind(XboxOne.lb, () -> System.out.println("lb"));
		quickBind(XboxOne.rb, () -> System.out.println("rb"));
		quickBind(XboxOne.lt, () -> System.out.println("lt"));
		quickBind(XboxOne.rt, () -> System.out.println("rt"));

		quickBind(POV.DPadUp, () -> System.out.println("dpad up"));
		quickBind(POV.DPadUpRight, () -> System.out.println("dpad up right"));
		quickBind(POV.DPadRight, () -> System.out.println("dpad right"));
		quickBind(POV.DPadDownRight, () -> System.out.println("dpad down right"));
		quickBind(POV.DPadDown, () -> System.out.println("dpad down"));
		quickBind(POV.DPadDownLeft, () -> System.out.println("dpad down left"));
		quickBind(POV.DPadLeft, () -> System.out.println("dpad left"));
		quickBind(POV.DPadUpLeft, () -> System.out.println("dpad up left"));

		quickBind(POV.Lt, () -> System.out.println("POV lt"));
		quickBind(POV.Rt, () -> System.out.println("POV rt"));
		return tmp_bindings;
	}
}
