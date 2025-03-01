package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberEncoderZero extends Command {
    private final ClimberSubsystem climber;
    private Timer timer = new Timer();
    private boolean mIsFinished = false;

    public ClimberEncoderZero(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        mIsFinished = false;
        climber.setMotorSpeed(-0.1);
    }


    @Override
    public void execute() {
        if (timer.get() > 0.5 && climber.getAmperage() > 0.045) {
            mIsFinished = true;
          }
    }

    public void end(boolean interrupted) {
        climber.setMotorSpeed(0);
        System.out.println("ClimberEncoderZero ended");
        timer.stop();
        climber.resetEncoder();
        climber.setPositionForward(Constants.ClimberSubsystem.positionSteady);
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}
