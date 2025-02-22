package frc.robot.subsystems;


import org.opencv.core.Mat;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class CoralDispenserSubsystem extends SubsystemBase {
    private FridolinsMotor coralMotorTop;
    private FridoSparkMax coralMotorChangePitch;    

    private PidValues pidValuesPitch = Constants.CoralDispenser.PidValuesPitch;
    private MAXMotionConfig smartMotionConfig;
    private SparkMaxConfig motorConfig;
    private double desiredPitch;
    
    public CoralDispenserSubsystem() {
        desiredPitch = 0;
        motorConfig = new SparkMaxConfig();
        smartMotionConfig = new MAXMotionConfig();
        coralMotorTop = new FridoSparkMax(Constants.CoralDispenser.CoralEndEffectorMotorID);
        
        coralMotorChangePitch = new FridoSparkMax(Constants.CoralDispenser.coralPitchMotorID);

        coralMotorChangePitch.setIdleMode(IdleMode.kBrake);;

        SparkMaxConfig limitConfig = new SparkMaxConfig(); //TODO Calibrate the limits after setting absolute encoder Offsets
        limitConfig.softLimit
        .forwardSoftLimit(Constants.CoralDispenser.pitchMotorForwardLimit).forwardSoftLimitEnabled(true);
        limitConfig.softLimit
        .forwardSoftLimit(Constants.CoralDispenser.pitchMotorReverseForwardLimit).reverseSoftLimitEnabled(true);

        coralMotorChangePitch.asSparkMax().configure(limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        smartMotionConfig.allowedClosedLoopError(Constants.CoralDispenser.kAllowedClosedLoopError, ClosedLoopSlot.kSlot0);
        smartMotionConfig.maxAcceleration(Constants.CoralDispenser.kMaxAcceleration, ClosedLoopSlot.kSlot0);
        smartMotionConfig.maxVelocity(Constants.CoralDispenser.kMaxVelocity, ClosedLoopSlot.kSlot0);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

        motorConfig.closedLoop.maxMotion.apply(smartMotionConfig);

        motorConfig.closedLoop.p(pidValuesPitch.kP, ClosedLoopSlot.kSlot0).i(pidValuesPitch.kI, ClosedLoopSlot.kSlot0)
            .d(pidValuesPitch.kD, ClosedLoopSlot.kSlot0)
            .outputRange(pidValuesPitch.peakOutputReverse, pidValuesPitch.peakOutputForward, ClosedLoopSlot.kSlot0)
            .velocityFF(pidValuesPitch.kF.orElse(0.0), ClosedLoopSlot.kSlot0);

        motorConfig.smartCurrentLimit(30, 30);

        coralMotorChangePitch.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        
        resetPitchEncoder();

        // coralMotorTop.setPID(pidValuesMotorTop);
        // coralMotorTop.enableForwardLimitSwitch(Constants.CoralDispenser.fwdMotorTopPolarity, true);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Absolute Rotation", getAbsoluteRotation() * 360);
        SmartDashboard.putNumber("Motor Encoder Thicks", coralMotorChangePitch.getEncoderTicks());
        SmartDashboard.putBoolean("Is at goal Encoder Thicks", isAtDesiredPitch());
        SmartDashboard.putBoolean("LimitSwitch", isForwardLimitSwitchPressedMotorTop());
    }



    public void setPitchPercent(double percent) {
        coralMotorChangePitch.set(percent);
    }

    public void resetPitchEncoder() {
        coralMotorChangePitch.setEncoderPosition(getAbsoluteRotation() * Constants.CoralDispenser.kArmGearRatio);
    }

    public double getAbsoluteRotation(){
        return coralMotorChangePitch.asSparkMax().getAbsoluteEncoder().getPosition() - Constants.CoralDispenser.angularOffset;
    }

    public boolean isForwardLimitSwitchPressedMotorTop() {
        return !coralMotorTop.isForwardLimitSwitchActive();
    }

    public void setMotorTopSpeed(double speed) {
        /*if (speed > 1 || speed < -1) {  ---> Do we need that really? It is in shooter and I think when percent output is 1.2 it automatically makes it 1
            throw new IllegalArgumentException("Speed must be between -1 and 1");
        } */
        coralMotorTop.set(speed);
    }

    public void stopMotorTop() {
        coralMotorTop.set(Constants.CoralDispenser.stopSpeedMotorTop);
    }

    public void stopMotorPitch() {
        coralMotorChangePitch.set(Constants.CoralDispenser.stopSpeedPitch);
    }

    public boolean isAtDesiredPitch(){
        return Math.abs(desiredPitch - coralMotorChangePitch.getEncoderTicks()) <= Constants.CoralDispenser.kAllowedClosedLoopError;//Tolerance is for now 0.5 thicks later can be changed
    }

    /**
     * 
     * @param pitch in encoder ticks
     */
    public void setPitch(double pitch) {
        //Rotation2d rotation = Rotation2d.fromDegrees(pitch);
        desiredPitch = pitch;
        coralMotorChangePitch.asSparkMax().getClosedLoopController().setReference(pitch, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Abs Encoder Coral Dispenser", () -> getAbsoluteRotation() * 360, null);
        builder.addDoubleProperty("Motor Encoder Coral Dispenser", () -> coralMotorChangePitch.getEncoderTicks(), null);
        builder.addBooleanProperty("is at desired pitch", () -> this.isAtDesiredPitch(), null);
        builder.addDoubleProperty("desired pitch", () -> desiredPitch, null);
        super.initSendable(builder);
    }
}
