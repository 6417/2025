package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class ClimberSubsytem extends SubsystemBase {
    private FridoSparkMax climberMotor;

    private PidValues pidValuesOut = Constants.ClimberSubsytem.PidValuesOutClimberSubsystem;
    private PidValues pidValuesIn = Constants.ClimberSubsytem.PidValuesInClimberSubsystem;

    private SparkMaxConfig motorConfig;
    private MAXMotionConfig smartMotionConfig;
    
    public ClimberSubsytem() {
        climberMotor = new FridoSparkMax(Constants.ClimberSubsytem.climberMotorID);

        motorConfig = new SparkMaxConfig();
        smartMotionConfig = new MAXMotionConfig();

        // slot 0 for climber out
        // slot 1 for climber back in

        motorConfig.closedLoop.p(pidValuesOut.kP, ClosedLoopSlot.kSlot0).i(pidValuesOut.kI, ClosedLoopSlot.kSlot0)
            .d(pidValuesOut.kD, ClosedLoopSlot.kSlot0)
            .outputRange(pidValuesOut.peakOutputReverse, pidValuesOut.peakOutputForward, ClosedLoopSlot.kSlot0)
            .velocityFF(pidValuesOut.kF.orElse(0.0), ClosedLoopSlot.kSlot0);
        
        pidValuesOut.iZone.ifPresent(iZone -> motorConfig.closedLoop.iZone(iZone, ClosedLoopSlot.kSlot0));

        motorConfig.closedLoop.p(pidValuesIn.kP, ClosedLoopSlot.kSlot1).i(pidValuesIn.kI, ClosedLoopSlot.kSlot1)
            .d(pidValuesIn.kD, ClosedLoopSlot.kSlot1)
            .outputRange(pidValuesIn.peakOutputReverse, pidValuesIn.peakOutputForward, ClosedLoopSlot.kSlot1)
            .velocityFF(pidValuesIn.kF.orElse(0.0), ClosedLoopSlot.kSlot1);
        
        pidValuesIn.iZone.ifPresent(iZone -> motorConfig.closedLoop.iZone(iZone, ClosedLoopSlot.kSlot1));

        
        smartMotionConfig.allowedClosedLoopError(Constants.ClimberSubsytem.kAllowedClosedLoopErrorOut, ClosedLoopSlot.kSlot0);
        smartMotionConfig.maxAcceleration(Constants.ClimberSubsytem.kMaxAccelerationOut, ClosedLoopSlot.kSlot0);
        smartMotionConfig.maxVelocity(Constants.ClimberSubsytem.kMaxVelocityOut, ClosedLoopSlot.kSlot0);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

        smartMotionConfig.allowedClosedLoopError(Constants.ClimberSubsytem.kAllowedClosedLoopErrorIn, ClosedLoopSlot.kSlot1);
        smartMotionConfig.maxAcceleration(Constants.ClimberSubsytem.kMaxAccelerationIn, ClosedLoopSlot.kSlot1);
        smartMotionConfig.maxVelocity(Constants.ClimberSubsytem.kMaxVelocityIn, ClosedLoopSlot.kSlot1);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1);
        
        climberMotor.asSparkMax().getClosedLoopController().setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        motorConfig.closedLoop.maxMotion.apply(smartMotionConfig);
      
        climberMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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

    public void setPositionUnderLoad(double position) {
        climberMotor.asSparkMax().getClosedLoopController().setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        climberMotor.setPosition(position);

    }
    public void setPositionForward(double position) {
        climberMotor.asSparkMax().getClosedLoopController().setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        climberMotor.setPosition(position);
    }
}