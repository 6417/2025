package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class ClimberSubsytem extends SubsystemBase {
    private FridoSparkMax climberMotorR;
    private FridoSparkMax climberMotorL;

    private PidValues pidValues = Constants.ClimberSubsytem.PidValuesClimberSubsystem;
    
    public ClimberSubsytem() {
        climberMotorR = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorRID);
        climberMotorL = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorLID);

        climberMotorL.follow(climberMotorR, DirectionType.invertMaster);

        climberMotorR.setPID(pidValues);
    }

    public void setSpeed(double speed) {
        climberMotorR.set(speed);
    }

    public void resetEncoder() {
        climberMotorR.setEncoderPosition(Constants.ClimberSubsytem.resetPitchEncoderPosition);
    }

    public void setMotorSpeed(double speed) {
        if (speed > 1 || speed < -1) {
            throw new IllegalArgumentException("Speed must be between -1 and 1");
        }
        climberMotorR.set(speed);
    }

    public void setPosition(double position) {
        climberMotorR.setPosition(position);
    }
    

}
