package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class CoralDispenserSubsystem extends SubsystemBase {
    private FridolinsMotor coralMotorTop;
    private FridolinsMotor coralMotorChangePitch;

    private PidValues pidValuesPitch = Constants.CoralDispenser.PidValuesPitch;
    private PidValues pidValuesMotorTop = Constants.CoralDispenser.PidValuesMotorTop;
    
    public CoralDispenserSubsystem() {
        coralMotorTop = new FridoSparkMax(Constants.CoralDispenser.coralMotorTopID);
        
        coralMotorChangePitch = new FridoSparkMax(Constants.ClimberSubsytem.coralMotorChangePitchID);
        
        coralMotorChangePitch.setPID(pidValuesPitch);
        coralMotorChangePitch.enableReverseLimitSwitch(Constants.CoralDispenser.revPolarity, true);
        coralMotorChangePitch.enableForwardLimitSwitch(Constants.CoralDispenser.fwdPolarity, true);

        coralMotorTop.setPID(pidValuesMotorTop);
        coralMotorTop.enableForwardLimitSwitch(Constants.CoralDispenser.fwdMotorTopPolarity, true);

    }

    public void setPitchPercent(double percent) {
        coralMotorChangePitch.set(percent);
    }

    public void resetPitchEncoder() {
        coralMotorChangePitch.setEncoderPosition(Constants.CoralDispenser.resetPitchEncoderPosition);
    }

    public void resetMotorTopEncoder() {
        coralMotorTop.setEncoderPosition(Constants.CoralDispenser.resetMotorTopEncoderPosition);
    }

    public boolean isForwardLimitSwitchPressed() {
        return coralMotorChangePitch.isForwardLimitSwitchActive();
    }

    public boolean isReverseLimitSwitchPressed() {
        return coralMotorChangePitch.isReverseLimitSwitchActive();
    }

    public void setMotorTopSpeed(double speed) {
        if (speed > 1 || speed < -1) {
            throw new IllegalArgumentException("Speed must be between -1 and 1");
        }
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
}
