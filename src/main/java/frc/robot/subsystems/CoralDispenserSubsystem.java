package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    
    public CoralDispenserSubsystem() {
        coralMotorTop = new FridoSparkMax(Constants.CoralDispenser.CoralEndEffectorMotorID);
        
        coralMotorChangePitch = new FridoSparkMax(Constants.CoralDispenser.coralPitchMotorID);
        coralMotorChangePitch.asSparkMax().getAbsoluteEncoder();
        coralMotorChangePitch.setPID(pidValuesPitch);

        coralMotorChangePitch.setIdleMode(IdleMode.kCoast);;

        SparkMaxConfig limitConfig = new SparkMaxConfig(); //TODO Calibrate the limits after setting absolute encoder Offsets
        limitConfig.softLimit
        .forwardSoftLimit(Constants.CoralDispenser.pitchMotorForwardLimit).forwardSoftLimitEnabled(true);
        limitConfig.softLimit
        .forwardSoftLimit(Constants.CoralDispenser.pitchMotorReverseForwardLimit).reverseSoftLimitEnabled(true);

        coralMotorChangePitch.asSparkMax().configure(limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        resetPitchEncoder();

        // coralMotorTop.setPID(pidValuesMotorTop);
        // coralMotorTop.enableForwardLimitSwitch(Constants.CoralDispenser.fwdMotorTopPolarity, true);

    }

    public void periodic() {
        System.out.println(coralMotorChangePitch.asSparkMax().getAbsoluteEncoder().getPosition()*360);
        
    }



    public void setPitchPercent(double percent) {
        coralMotorChangePitch.set(percent);
    }

    public void resetPitchEncoder() {
        coralMotorChangePitch.setEncoderPosition((coralMotorChangePitch.asSparkMax().getAbsoluteEncoder().getPosition() - Constants.CoralDispenser.angularOffset) * Constants.CoralDispenser.kArmGearRatio);
    }

    public boolean isForwardLimitSwitchPressedMotorTop() {
        return coralMotorTop.isForwardLimitSwitchActive();
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

    /**
     * 
     * @param pitch in degrees
     */
    public void setPitch(double pitch) {
        Rotation2d rotation = Rotation2d.fromDegrees(pitch);
        coralMotorChangePitch.setPosition(rotation.getRotations());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Abs Encoder Coral Dispenser", () -> coralMotorChangePitch.asSparkMax().getAbsoluteEncoder().getPosition(), null);
        super.initSendable(builder);
    }
}
