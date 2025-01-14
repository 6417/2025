package frc.robot.subsystems;

import frc.fridowpi.motors.FridoServoMotor;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;

public class ClimberSubsytem {
    private ClimberSubsytem instance;
    
    private FridoSparkMax climberMotorR;
    private FridoSparkMax climberMotorL;
    
    public ClimberSubsytem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsytem();
        }

        return instance;
    }

    private ClimberSubsytem() {
        climberMotorR = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorRID);
        climberMotorL = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorLID);
    }

}
