package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;
import frc.robot.RobotContainer;
public class LiftingTowerSubsystem extends SubsystemBase {
    private final PidValues pidValues = new PidValues(0, 0, 0, 0); // p, i, d, f
            
    private FridolinsMotor motorLeft;
    private FridolinsMotor motorRight;

    public LiftingTowerSubsystem() {
        motorLeft = new FridoSparkMax(Constants.LiftingTower.liftingTowerLeftId);
        motorRight = new FridoSparkMax(Constants.LiftingTower.liftingTowerRightId);
        
        motorLeft.follow(motorRight, DirectionType.invertMaster);

        motorRight.enableForwardLimitSwitch(Constants.LiftingTower.fdwLiftingTowePolarity, true);

        Encoder encoder = new Encoder(0, 1);
        encoder.getDistance();

        motorRight.setPID(pidValues);

        

    }

    public void resetEncoder() {
        motorRight.setEncoderPosition(Constants.LiftingTower.resetEncoderPosition);
    }

    public boolean isForwardLimitSwitchPressed() {
        return motorRight.isForwardLimitSwitchActive();
    }



    public void setPercent(double percent) {
        motorRight.set(percent);
    }
    
    public void setMotorSpeed(double speed) {
        if (speed < -1.0 || speed > 1.0) {
            throw new IllegalArgumentException("Set speed between -1.0 and 1.0");
        }

        motorRight.set(speed);
    }

    public void setHeight(double position) {
        motorRight.setPosition(position);
    }

    public void stopMotors() {
        motorRight.set(Constants.LiftingTower.stopSpeed);
    }
}
