package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private FridoSparkMax climberMotor;

    private PidValues pidValuesOut = Constants.ClimberSubsystem.PidValuesOutClimberSubsystem;
    private PidValues pidValuesIn = Constants.ClimberSubsystem.PidValuesInClimberSubsystem;

    private SparkMaxConfig motorConfig;
    private MAXMotionConfig smartMotionConfig;
    
    public ClimberSubsystem() {
        climberMotor = new FridoSparkMax(Constants.ClimberSubsystem.climberMotorID);
        
        reconfigure();

        InstantCommand resetEncoderToZero = new InstantCommand(()->{
            climberMotor.setEncoderPosition(0);
        });
        
        resetEncoderToZero.schedule();
        Shuffleboard.getTab("Climber").add("resetEncoder", resetEncoderToZero);
    }

    private void reconfigure() {
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

        
        smartMotionConfig.allowedClosedLoopError(Constants.ClimberSubsystem.kAllowedClosedLoopErrorOut, ClosedLoopSlot.kSlot0);
        smartMotionConfig.maxAcceleration(Constants.ClimberSubsystem.kMaxAccelerationOut, ClosedLoopSlot.kSlot0);
        smartMotionConfig.maxVelocity(Constants.ClimberSubsystem.kMaxVelocityOut, ClosedLoopSlot.kSlot0);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

        smartMotionConfig.allowedClosedLoopError(Constants.ClimberSubsystem.kAllowedClosedLoopErrorIn, ClosedLoopSlot.kSlot1);
        smartMotionConfig.maxAcceleration(Constants.ClimberSubsystem.kMaxAccelerationIn, ClosedLoopSlot.kSlot1);
        smartMotionConfig.maxVelocity(Constants.ClimberSubsystem.kMaxVelocityIn, ClosedLoopSlot.kSlot1);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1);
        
        climberMotor.asSparkMax().getClosedLoopController().setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        motorConfig.closedLoop.maxMotion.apply(smartMotionConfig);
      
        climberMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void resetEncoder() {
        climberMotor.setEncoderPosition(Constants.ClimberSubsystem.resetPitchEncoderPosition);
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

    @Override    
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kP in", () -> Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kP, 
                (double pid) -> {Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kP = pid;
                reconfigure();
            }); 
            
        builder.addDoubleProperty("kI in", () -> Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kI, 
                (double pid) -> {Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kI = pid;
                reconfigure();
            }); 
            
        builder.addDoubleProperty("kD in", () -> Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kD, 
                (double pid) -> {Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kD = pid;
                reconfigure();
            }); 
            
            
        builder.addDoubleProperty("kP out", () -> Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kP, 
                (double pid) -> {Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kP = pid;
                reconfigure();
            }); 
            
        builder.addDoubleProperty("kI out", () -> Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kI, 
                (double pid) -> {Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kI = pid;
                reconfigure();
            }); 
            
        builder.addDoubleProperty("kD out", () -> Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kD, 
                (double pid) -> {Constants.ClimberSubsystem.PidValuesInClimberSubsystem.kD = pid;
                reconfigure();
            }); 

            
            
        builder.addDoubleProperty("kAllowedClosedLoopErrorIn", () -> Constants.ClimberSubsystem.kAllowedClosedLoopErrorIn, 
                (double val) -> {Constants.ClimberSubsystem.kAllowedClosedLoopErrorIn = val;
                reconfigure();
            });
            
            
        builder.addDoubleProperty("kAllowedClosedLoopErrorOut", () -> Constants.ClimberSubsystem.kAllowedClosedLoopErrorOut, 
                (double val) -> {Constants.ClimberSubsystem.kAllowedClosedLoopErrorOut = val;
                reconfigure();
            });


            
        builder.addDoubleProperty("kMaxAccelerationIn", () -> Constants.ClimberSubsystem.kMaxAccelerationIn, 
                (double val) -> {Constants.ClimberSubsystem.kMaxAccelerationIn = val;
                reconfigure();
            });

            
        builder.addDoubleProperty("kMaxAccelerationOut", () -> Constants.ClimberSubsystem.kMaxAccelerationOut, 
                (double val) -> {
                Constants.ClimberSubsystem.kMaxAccelerationOut = val;
                reconfigure();
            });


            
        builder.addDoubleProperty("kMaxVelocityIn", () -> Constants.ClimberSubsystem.kMaxVelocityIn, 
                (double val) -> {
                Constants.ClimberSubsystem.kMaxVelocityIn = val;
                reconfigure();
            });

            
        builder.addDoubleProperty("kMaxVelocityOut", () -> Constants.ClimberSubsystem.kMaxVelocityOut, 
                (double val) -> {Constants.ClimberSubsystem.kMaxVelocityOut = val;
                reconfigure();
            });


            
        builder.addDoubleProperty("positionFront", () -> Constants.ClimberSubsystem.positionFront, 
                (double val) -> {Constants.ClimberSubsystem.positionFront = val;
                reconfigure();
            });

            
        builder.addDoubleProperty("positionBack", () -> Constants.ClimberSubsystem.positionBack, 
                (double val) -> {Constants.ClimberSubsystem.positionBack = val;
                reconfigure();
            });
            
            
        super.initSendable(builder);
    }
}
