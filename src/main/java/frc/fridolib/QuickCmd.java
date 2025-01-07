package frc.fridolib;

import java.util.function.BooleanSupplier;

import frc.fridowpi.command.FridoCommand;

/**
 * QuickCommands
 */
public class QuickCmd {

	public static FridoCommand withInit(Runnable init) {
		return new FridoCommand() {
			@Override
			public void initialize() {
				init.run();
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}

	public static FridoCommand periodic(Runnable periodic) {
		return new FridoCommand() {
			@Override
			public void execute() {
				periodic.run();
			}
		};
	}

	public static FridoCommand runUntil(BooleanSupplier finish) {
		return new FridoCommand() {
			@Override
			public boolean isFinished() {
				return finish.getAsBoolean();
			}
		};
	}

	// Familiar, but less logical names
	public static FridoCommand isFinished(BooleanSupplier finish) {
		return new FridoCommand() {
			@Override
			public boolean isFinished() {
				return finish.getAsBoolean();
			}
		};
	}
}
