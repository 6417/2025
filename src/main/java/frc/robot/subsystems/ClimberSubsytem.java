package frc.robot.subsystems;

import frc.fridowpi.motors.FridoServoMotor;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;

public class ClimberSubsytem {
    private FridoSparkMax climberMotorR;
    private FridoSparkMax climberMotorL;
    
    public ClimberSubsytem() {
        climberMotorR = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorRID);
        climberMotorL = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorLID);
    }

    public void periodic() {
    }

    public void simulationPeriodic() {
    }
}
