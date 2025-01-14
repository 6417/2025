package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.fridowpi.motors.FridoServoMotor;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class ClimberSubsytem {
    private FridoSparkMax climberMotorR;
    private FridoSparkMax climberMotorL;

    private Rotation2d rotation2d;

    private PidValues pidValues = new PidValues(0, 0, 0, 0); // p, i, d, f
    
    public ClimberSubsytem() {
        climberMotorR = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorRID);
        climberMotorL = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorLID);

        climberMotorL.follow(climberMotorR, DirectionType.invertMaster);

        climberMotorR.setPID(pidValues);
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
