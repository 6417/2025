package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class ClimberSubsytem extends SubsystemBase {
    private FridoSparkMax climberMotor;

    private PidValues pidValues = Constants.ClimberSubsytem.PidValuesClimberSubsystem;
    
    public ClimberSubsytem() {
        climberMotor = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorID);

        climberMotor.setPID(pidValues);
    }

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void resetEncoder() {
        climberMotor.setEncoderPosition(Constants.ClimberSubsytem.resetPitchEncoderPosition);
    }

    public void setMotorSpeed(double speed) {
        if (speed > 1 || speed < -1) {
            throw new IllegalArgumentException("Speed must be between -1 and 1");
        }
        climberMotor.set(speed);
    }

    public void setPosition(double position) {
        climberMotor.setPosition(position);
    }
    

}
