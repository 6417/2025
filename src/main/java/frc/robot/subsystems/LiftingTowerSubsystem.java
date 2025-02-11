package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class LiftingTowerSubsystem extends SubsystemBase {
    private final PidValues pidValues = Constants.LiftingTower.pidValues; // p, i, d, f
            
    private SparkMaxConfig motorConfig;

    private FridoSparkMax motorLeft;
    private FridoSparkMax motorRight;

    private MAXMotionConfig smartMotionConfig;
    
    public LiftingTowerSubsystem() {
        motorLeft = new FridoSparkMax(Constants.LiftingTower.liftingTowerLeftId);
        motorRight = new FridoSparkMax(Constants.LiftingTower.liftingTowerRightId);
        
        motorLeft.follow(motorRight, DirectionType.invertMaster);

        motorConfig = new SparkMaxConfig();
        smartMotionConfig = new MAXMotionConfig();

        motorRight.enableForwardLimitSwitch(Constants.LiftingTower.fdwLiftingTowePolarity, true);

        motorConfig.closedLoop.p(pidValues.kP).i(pidValues.kI).d(pidValues.kD).outputRange(pidValues.peakOutputReverse,
        pidValues.peakOutputForward).velocityFF(pidValues.kF.orElse(0.0));
        pidValues.iZone.ifPresent(iZone -> motorConfig.closedLoop.iZone(iZone));

        smartMotionConfig.allowedClosedLoopError(Constants.LiftingTower.kAllowedClosedLoopError);
        smartMotionConfig.maxAcceleration(Constants.LiftingTower.kMaxAcceleration);
        smartMotionConfig.maxVelocity(Constants.LiftingTower.kMaxVelocity);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        motorConfig.closedLoop.maxMotion.apply(smartMotionConfig);
      
        motorRight.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
