package frc.robot.subsystems;

import java.nio.channels.UnsupportedAddressTypeException;
import java.util.function.LongFunction;
import java.util.logging.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class LiftingTowerSubsystem extends SubsystemBase {
    private final PidValues pidValues = new PidValues(0, 0, 0, 0); // p, i, d, f
            
    private FridoSparkMax motorLeft;
    private FridoSparkMax motorRight;
    
    public LiftingTowerSubsystem() {
        motorLeft = new FridoSparkMax(Constants.LiftingTower.liftingTowerLeftId);
        motorRight = new FridoSparkMax(Constants.LiftingTower.liftingTowerRightId);
        
        motorLeft.follow(motorRight, DirectionType.invertMaster);
        
        motorRight.setPID(pidValues);
    }
    
    public void startMotors(double speed) {
        if (speed < -1.0 || speed > 1.0) {
            throw new IllegalArgumentException("Set speed between -1.0 and 1.0");
        }

        motorRight.set(speed);
    }

    public void setPosition(double position) {
        motorRight.setPosition(position);
    }

    public void stopMotors() {
        startMotors(0);
    }
}
